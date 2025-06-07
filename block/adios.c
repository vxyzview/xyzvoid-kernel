// SPDX-License-Identifier: GPL-2.0
/*
 * Adaptive Deadline I/O Scheduler (ADIOS)
 * Copyright (C) 2025 Masahito Suzuki
 */
#include <linux/bio.h>
#include <linux/blkdev.h>
#include <linux/compiler.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/rbtree.h>
#include <linux/sbitmap.h>
#include <linux/slab.h>
#include <linux/timekeeping.h>

#include "elevator.h"
#include "blk.h"
#include "blk-mq.h"
#include "blk-mq-sched.h"

#define ADIOS_VERSION "1.5.8"

// Define operation types supported by ADIOS
enum adios_op_type {
	ADIOS_READ    = 0,
	ADIOS_WRITE   = 1,
	ADIOS_DISCARD = 2,
	ADIOS_OTHER   = 3,
	ADIOS_OPTYPES = 4,
};

// Global variable to control the latency
static u64 default_global_latency_window = 16000000ULL;
// Ratio below which batch queues should be refilled
static u8  default_bq_refill_below_ratio = 15;

// Dynamic thresholds for shrinkage
static u32 default_lm_shrink_at_kreqs  = 10000;
static u32 default_lm_shrink_at_gbytes =   100;
static u32 default_lm_shrink_resist    =     2;

// Latency targets for each operation type
static u64 default_latency_target[ADIOS_OPTYPES] = {
	[ADIOS_READ]    =     1ULL * NSEC_PER_MSEC,
	[ADIOS_WRITE]   =  2000ULL * NSEC_PER_MSEC,
	[ADIOS_DISCARD] =  8000ULL * NSEC_PER_MSEC,
	[ADIOS_OTHER]   =     0ULL * NSEC_PER_MSEC,
};

// Maximum batch size limits for each operation type
static u32 default_batch_limit[ADIOS_OPTYPES] = {
	[ADIOS_READ]    = 24,
	[ADIOS_WRITE]   = 48,
	[ADIOS_DISCARD] =  1,
	[ADIOS_OTHER]   =  1,
};

static u32 default_dl_prio[2] = {7, 0};

// Thresholds for latency model control
#define LM_BLOCK_SIZE_THRESHOLD 4096
#define LM_SAMPLES_THRESHOLD    1024
#define LM_INTERVAL_THRESHOLD   1500
#define LM_OUTLIER_PERCENTILE     99
#define LM_LAT_BUCKET_COUNT       64

// Structure to hold latency bucket data for small requests
struct latency_bucket_small {
	u64 sum_latency;
	u32 count;
};

// Structure to hold latency bucket data for large requests
struct latency_bucket_large {
	u64 sum_latency;
	u64 sum_block_size;
	u32 count;
};

// Structure to hold the latency model context data
struct latency_model {
	spinlock_t lock;
	u64 base;
	u64 slope;
	u64 small_sum_delay;
	u64 small_count;
	u64 large_sum_delay;
	u64 large_sum_bsize;
	u64 last_update_jiffies;

	spinlock_t buckets_lock;
	struct latency_bucket_small small_bucket[LM_LAT_BUCKET_COUNT];
	struct latency_bucket_large large_bucket[LM_LAT_BUCKET_COUNT];

	u32 lm_shrink_at_kreqs;
	u32 lm_shrink_at_gbytes;
	u8  lm_shrink_resist;
};

#define ADIOS_BQ_PAGES 2

// Adios scheduler data
struct adios_data {
	spinlock_t pq_lock;
	struct list_head prio_queue;

	struct rb_root_cached dl_tree[2];
	spinlock_t lock;
	u8  dl_queued;
	s64 dl_bias;
	s32 dl_prio[2];

	u64 global_latency_window;
	u64 latency_target[ADIOS_OPTYPES];
	u32 batch_limit[ADIOS_OPTYPES];
	u32 batch_actual_max_size[ADIOS_OPTYPES];
	u32 batch_actual_max_total;
	u32 async_depth;
	u8  bq_refill_below_ratio;

	u8 bq_page;
	bool more_bq_ready;
	struct list_head batch_queue[ADIOS_BQ_PAGES][ADIOS_OPTYPES];
	u32 batch_count[ADIOS_BQ_PAGES][ADIOS_OPTYPES];
	spinlock_t bq_lock;

	struct latency_model latency_model[ADIOS_OPTYPES];
	struct timer_list update_timer;

	atomic64_t total_pred_lat;

	struct kmem_cache *rq_data_pool;
	struct kmem_cache *dl_group_pool;

	struct request_queue *queue;
};

// List of requests with the same deadline in the deadline-sorted tree
struct dl_group {
	struct rb_node node;
	struct list_head rqs;
	u64 deadline;
} __attribute__((aligned(64)));

// Structure to hold scheduler-specific data for each request
struct adios_rq_data {
	struct list_head *dl_group;
	struct list_head dl_node;

	struct request *rq;
	u64 deadline;
	u64 pred_lat;
	u32 block_size;
} __attribute__((aligned(64)));

static const int adios_prio_to_weight[40] = {
 /* -20 */     88761,     71755,     56483,     46273,     36291,
 /* -15 */     29154,     23254,     18705,     14949,     11916,
 /* -10 */      9548,      7620,      6100,      4904,      3906,
 /*  -5 */      3121,      2501,      1991,      1586,      1277,
 /*   0 */      1024,       820,       655,       526,       423,
 /*   5 */       335,       272,       215,       172,       137,
 /*  10 */       110,        87,        70,        56,        45,
 /*  15 */        36,        29,        23,        18,        15,
};

// Count the number of entries in small buckets
static u32 lm_count_small_entries(struct latency_model *model) {
	u32 total_count = 0;
	for (u8 i = 0; i < LM_LAT_BUCKET_COUNT; i++)
		total_count += model->small_bucket[i].count;
	return total_count;
}

// Update the small buckets in the latency model
static bool lm_update_small_buckets(struct latency_model *model,
		u32 total_count, bool count_all) {
	u64 sum_latency = 0;
	u32 sum_count = 0;
	u32 cumulative_count = 0, threshold_count = 0;
	u8  outlier_threshold_bucket = 0;
	u8  outlier_percentile = LM_OUTLIER_PERCENTILE;
	u8  reduction;

	if (count_all)
		outlier_percentile = 100;

	// Calculate the threshold count for outlier detection
	threshold_count = (total_count * outlier_percentile) / 100;

	// Identify the bucket that corresponds to the outlier threshold
	for (u8 i = 0; i < LM_LAT_BUCKET_COUNT; i++) {
		cumulative_count += model->small_bucket[i].count;
		if (cumulative_count >= threshold_count) {
			outlier_threshold_bucket = i;
			break;
		}
	}

	// Calculate the average latency, excluding outliers
	for (u8 i = 0; i <= outlier_threshold_bucket; i++) {
		struct latency_bucket_small *bucket = &model->small_bucket[i];
		if (i < outlier_threshold_bucket) {
			sum_latency += bucket->sum_latency;
			sum_count += bucket->count;
		} else {
			// The threshold bucket's contribution is proportional
			u64 remaining_count =
				threshold_count - (cumulative_count - bucket->count);
			if (bucket->count > 0) {
				sum_latency +=
					div_u64((bucket->sum_latency * remaining_count), bucket->count);
				sum_count += remaining_count;
			}
		}
	}

	// Shrink the model if it reaches at the readjustment threshold
	if (model->small_count >= 1000ULL * model->lm_shrink_at_kreqs) {
		reduction = model->lm_shrink_resist;
		if (model->small_count >> reduction) {
			model->small_sum_delay -= model->small_sum_delay >> reduction;
			model->small_count     -= model->small_count     >> reduction;
		}
	}

	// Accumulate the average latency into the statistics
	model->small_sum_delay += sum_latency;
	model->small_count += sum_count;

	// Reset small bucket information
	memset(model->small_bucket, 0,
		sizeof(model->small_bucket[0]) * LM_LAT_BUCKET_COUNT);

	return true;
}

// Count the number of entries in large buckets
static u32 lm_count_large_entries(struct latency_model *model) {
	u32 total_count = 0;
	for (u8 i = 0; i < LM_LAT_BUCKET_COUNT; i++)
		total_count += model->large_bucket[i].count;
	return total_count;
}

// Update the large buckets in the latency model
static bool lm_update_large_buckets(
		struct latency_model *model,
		u32 total_count, bool count_all) {
	s64 sum_latency = 0;
	u64 sum_block_size = 0, intercept;
	u32 cumulative_count = 0, threshold_count = 0;
	u8  outlier_threshold_bucket = 0;
	u8  outlier_percentile = LM_OUTLIER_PERCENTILE;
	u8  reduction;

	if (count_all)
		outlier_percentile = 100;

	// Calculate the threshold count for outlier detection
	threshold_count = (total_count * outlier_percentile) / 100;

	// Identify the bucket that corresponds to the outlier threshold
	for (u8 i = 0; i < LM_LAT_BUCKET_COUNT; i++) {
		cumulative_count += model->large_bucket[i].count;
		if (cumulative_count >= threshold_count) {
			outlier_threshold_bucket = i;
			break;
		}
	}

	// Calculate the average latency and block size, excluding outliers
	for (u8 i = 0; i <= outlier_threshold_bucket; i++) {
		struct latency_bucket_large *bucket = &model->large_bucket[i];
		if (i < outlier_threshold_bucket) {
			sum_latency += bucket->sum_latency;
			sum_block_size += bucket->sum_block_size;
		} else {
			// The threshold bucket's contribution is proportional
			u64 remaining_count =
				threshold_count - (cumulative_count - bucket->count);
			if (bucket->count > 0) {
				sum_latency +=
					div_u64((bucket->sum_latency * remaining_count), bucket->count);
				sum_block_size +=
					div_u64((bucket->sum_block_size * remaining_count), bucket->count);
			}
		}
	}

	// Shrink the model if it reaches at the readjustment threshold
	if (model->large_sum_bsize >= 0x40000000ULL * model->lm_shrink_at_gbytes) {
		reduction = model->lm_shrink_resist;
		if (model->large_sum_bsize >> reduction) {
			model->large_sum_delay -= model->large_sum_delay >> reduction;
			model->large_sum_bsize -= model->large_sum_bsize >> reduction;
		}
	}

	// Accumulate the average delay into the statistics
	intercept = model->base * threshold_count;
	if (sum_latency > intercept)
		sum_latency -= intercept;

	model->large_sum_delay += sum_latency;
	model->large_sum_bsize += sum_block_size;

	// Reset large bucket information
	memset(model->large_bucket, 0,
		sizeof(model->large_bucket[0]) * LM_LAT_BUCKET_COUNT);

	return true;
}

// Update the latency model parameters and statistics
static void latency_model_update(struct latency_model *model) {
	unsigned long flags;
	u64 now;
	u32 small_count, large_count;
	bool time_elapsed;
	bool small_processed = false, large_processed = false;

	guard(spinlock_irqsave)(&model->lock);

	spin_lock_irqsave(&model->buckets_lock, flags);

	// Whether enough time has elapsed since the last update
	now = jiffies;
	time_elapsed = unlikely(!model->base) || model->last_update_jiffies +
		msecs_to_jiffies(LM_INTERVAL_THRESHOLD) <= now;

	// Count the number of entries in buckets
	small_count = lm_count_small_entries(model);
	large_count = lm_count_large_entries(model);

	// Update small buckets
	if (small_count && (time_elapsed ||
			LM_SAMPLES_THRESHOLD <= small_count || !model->base))
		small_processed = lm_update_small_buckets(
			model, small_count, !model->base);
	// Update large buckets
	if (large_count && (time_elapsed ||
			LM_SAMPLES_THRESHOLD <= large_count || !model->slope))
		large_processed = lm_update_large_buckets(
			model, large_count, !model->slope);

	spin_unlock_irqrestore(&model->buckets_lock, flags);

	// Update the base parameter if small bucket was processed
	if (small_processed && likely(model->small_count))
		model->base = div_u64(model->small_sum_delay, model->small_count);

	// Update the slope parameter if large bucket was processed
	if (large_processed && likely(model->large_sum_bsize))
		model->slope = div_u64(model->large_sum_delay,
			DIV_ROUND_UP_ULL(model->large_sum_bsize, 1024));

	// Reset statistics and update last updated jiffies if time has elapsed
	if (time_elapsed)
		model->last_update_jiffies = now;
}

// Determine the bucket index for a given measured and predicted latency
static u8 lm_input_bucket_index(
		struct latency_model *model, u64 measured, u64 predicted) {
	u8 bucket_index;

	if (measured < predicted * 2)
		bucket_index = div_u64((measured * 20), predicted);
	else if (measured < predicted * 5)
		bucket_index = div_u64((measured * 10), predicted) + 20;
	else
		bucket_index = div_u64((measured * 3), predicted) + 40;

	return bucket_index;
}

// Input latency data into the latency model
static void latency_model_input(struct latency_model *model,
		u32 block_size, u64 latency, u64 pred_lat) {
	unsigned long flags;
	u8 bucket_index;

	spin_lock_irqsave(&model->buckets_lock, flags);

	if (block_size <= LM_BLOCK_SIZE_THRESHOLD) {
		// Handle small requests
		bucket_index = lm_input_bucket_index(model, latency, model->base ?: 1);

		if (bucket_index >= LM_LAT_BUCKET_COUNT)
			bucket_index = LM_LAT_BUCKET_COUNT - 1;

		model->small_bucket[bucket_index].count++;
		model->small_bucket[bucket_index].sum_latency += latency;

		if (unlikely(!model->base)) {
			spin_unlock_irqrestore(&model->buckets_lock, flags);
			latency_model_update(model);
			return;
		}
	} else {
		// Handle large requests
		if (!model->base || !pred_lat) {
			spin_unlock_irqrestore(&model->buckets_lock, flags);
			return;
		}

		bucket_index = lm_input_bucket_index(model, latency, pred_lat);

		if (bucket_index >= LM_LAT_BUCKET_COUNT)
			bucket_index = LM_LAT_BUCKET_COUNT - 1;

		model->large_bucket[bucket_index].count++;
		model->large_bucket[bucket_index].sum_latency += latency;
		model->large_bucket[bucket_index].sum_block_size += block_size;
	}

	spin_unlock_irqrestore(&model->buckets_lock, flags);
}

// Predict the latency for a given block size using the latency model
static u64 latency_model_predict(struct latency_model *model, u32 block_size) {
	u64 result;

	guard(spinlock_irqsave)(&model->lock);
	// Predict latency based on the model
	result = model->base;
	if (block_size > LM_BLOCK_SIZE_THRESHOLD)
		result += model->slope *
			DIV_ROUND_UP_ULL(block_size - LM_BLOCK_SIZE_THRESHOLD, 1024);

	return result;
}

// Determine the type of operation based on request flags
static u8 adios_optype(struct request *rq) {
	switch (rq->cmd_flags & REQ_OP_MASK) {
	case REQ_OP_READ:
		return ADIOS_READ;
	case REQ_OP_WRITE:
		return ADIOS_WRITE;
	case REQ_OP_DISCARD:
		return ADIOS_DISCARD;
	default:
		return ADIOS_OTHER;
	}
}

static inline u8 adios_optype_not_read(struct request *rq) {
	return (rq->cmd_flags & REQ_OP_MASK) != REQ_OP_READ;
}

// Helper function to retrieve adios_rq_data from a request
static inline struct adios_rq_data *get_rq_data(struct request *rq) {
	return rq->elv.priv[0];
}

// Add a request to the deadline-sorted red-black tree
static void add_to_dl_tree(
		struct adios_data *ad, bool dl_idx, struct request *rq) {
	struct rb_root_cached *root = &ad->dl_tree[dl_idx];
	struct rb_node **link = &(root->rb_root.rb_node), *parent = NULL;
	bool leftmost = true;
	struct adios_rq_data *rd = get_rq_data(rq);
	struct dl_group *dlg;

	rd->block_size = blk_rq_bytes(rq);
	u8 optype = adios_optype(rq);
	rd->pred_lat =
		latency_model_predict(&ad->latency_model[optype], rd->block_size);
	rd->deadline =
		rq->start_time_ns + ad->latency_target[optype] + rd->pred_lat;

	while (*link) {
		dlg = rb_entry(*link, struct dl_group, node);
		s64 diff = rd->deadline - dlg->deadline;

		parent = *link;
		if (diff < 0) {
			link = &((*link)->rb_left);
		} else if (diff > 0) {
			link = &((*link)->rb_right);
			leftmost = false;
		} else { // diff == 0
			goto found;
		}
	}

	dlg = rb_entry_safe(parent, struct dl_group, node);
	if (!dlg || dlg->deadline != rd->deadline) {
		dlg = kmem_cache_zalloc(ad->dl_group_pool, GFP_ATOMIC);
		if (!dlg)
			return;
		dlg->deadline = rd->deadline;
		INIT_LIST_HEAD(&dlg->rqs);
		rb_link_node(&dlg->node, parent, link);
		rb_insert_color_cached(&dlg->node, root, leftmost);
	}
found:
	list_add_tail(&rd->dl_node, &dlg->rqs);
	rd->dl_group = &dlg->rqs;
	ad->dl_queued |= 1 << dl_idx;
}

// Remove a request from the deadline-sorted red-black tree
static void del_from_dl_tree(
		struct adios_data *ad, bool dl_idx, struct request *rq) {
	struct rb_root_cached *root = &ad->dl_tree[dl_idx];
	struct adios_rq_data *rd = get_rq_data(rq);
	struct dl_group *dlg = container_of(rd->dl_group, struct dl_group, rqs);

	list_del_init(&rd->dl_node);
	if (list_empty(&dlg->rqs)) {
		rb_erase_cached(&dlg->node, root);
		kmem_cache_free(ad->dl_group_pool, dlg);
	}
	rd->dl_group = NULL;

	if (RB_EMPTY_ROOT(&ad->dl_tree[dl_idx].rb_root))
		ad->dl_queued &= ~(1 << dl_idx);
}

// Remove a request from the scheduler
static void remove_request(struct adios_data *ad, struct request *rq) {
	bool dl_idx = adios_optype_not_read(rq);
	struct request_queue *q = rq->q;
	struct adios_rq_data *rd = get_rq_data(rq);

	list_del_init(&rq->queuelist);

	// We might not be on the rbtree, if we are doing an insert merge
	if (rd->dl_group)
		del_from_dl_tree(ad, dl_idx, rq);

	elv_rqhash_del(q, rq);
	if (q->last_merge == rq)
		q->last_merge = NULL;
}

// Convert a queue depth to the corresponding word depth for shallow allocation
static int to_word_depth(struct blk_mq_hw_ctx *hctx, unsigned int qdepth) {
	struct sbitmap_queue *bt = &hctx->sched_tags->bitmap_tags;
	const unsigned int nrr = hctx->queue->nr_requests;

	return ((qdepth << bt->sb.shift) + nrr - 1) / nrr;
}

// Limit the depth of request allocation for asynchronous and write requests
static void adios_limit_depth(blk_opf_t opf, struct blk_mq_alloc_data *data) {
	struct adios_data *ad = data->q->elevator->elevator_data;

	// Do not throttle synchronous reads
	if (op_is_sync(opf) && !op_is_write(opf))
		return;

	data->shallow_depth = to_word_depth(data->hctx, ad->async_depth);
}

// Update async_depth when the number of requests in the queue changes
static void adios_depth_updated(struct blk_mq_hw_ctx *hctx) {
	struct request_queue *q = hctx->queue;
	struct adios_data *ad = q->elevator->elevator_data;
	struct blk_mq_tags *tags = hctx->sched_tags;

	ad->async_depth = q->nr_requests;

	sbitmap_queue_min_shallow_depth(&tags->bitmap_tags, 1);
}

// Handle request merging after a merge operation
static void adios_request_merged(struct request_queue *q, struct request *req,
				  enum elv_merge type) {
	bool dl_idx = adios_optype_not_read(req);
	struct adios_data *ad = q->elevator->elevator_data;

	// if the merge was a front merge, we need to reposition request
	if (type == ELEVATOR_FRONT_MERGE) {
		del_from_dl_tree(ad, dl_idx, req);
		add_to_dl_tree(ad, dl_idx, req);
	}
}

// Handle merging of requests after one has been merged into another
static void adios_merged_requests(struct request_queue *q, struct request *req,
				   struct request *next) {
	struct adios_data *ad = q->elevator->elevator_data;

	lockdep_assert_held(&ad->lock);

	// kill knowledge of next, this one is a goner
	remove_request(ad, next);
}

// Try to merge a bio into an existing rq before associating it with an rq
static bool adios_bio_merge(struct request_queue *q, struct bio *bio,
		unsigned int nr_segs) {
	unsigned long flags;
	struct adios_data *ad = q->elevator->elevator_data;
	struct request *free = NULL;
	bool ret;

	spin_lock_irqsave(&ad->lock, flags);
	ret = blk_mq_sched_try_merge(q, bio, nr_segs, &free);
	spin_unlock_irqrestore(&ad->lock, flags);

	if (free)
		blk_mq_free_request(free);

	return ret;
}

// Insert a request into the scheduler
static void insert_request(struct blk_mq_hw_ctx *hctx, struct request *rq,
				  blk_insert_t insert_flags, struct list_head *free) {
	unsigned long flags;
	bool dl_idx = adios_optype_not_read(rq);
	struct request_queue *q = hctx->queue;
	struct adios_data *ad = q->elevator->elevator_data;

	lockdep_assert_held(&ad->lock);

	if (insert_flags & BLK_MQ_INSERT_AT_HEAD) {
		spin_lock_irqsave(&ad->pq_lock, flags);
		list_add(&rq->queuelist, &ad->prio_queue);
		spin_unlock_irqrestore(&ad->pq_lock, flags);
		return;
	}

	if (blk_mq_sched_try_insert_merge(q, rq, free))
		return;

	add_to_dl_tree(ad, dl_idx, rq);

	if (rq_mergeable(rq)) {
		elv_rqhash_add(q, rq);
		if (!q->last_merge)
			q->last_merge = rq;
	}
}

// Insert multiple requests into the scheduler
static void adios_insert_requests(struct blk_mq_hw_ctx *hctx,
				   struct list_head *list,
				   blk_insert_t insert_flags) {
	unsigned long flags;
	struct request_queue *q = hctx->queue;
	struct adios_data *ad = q->elevator->elevator_data;
	LIST_HEAD(free);

	spin_lock_irqsave(&ad->lock, flags);
	while (!list_empty(list)) {
		struct request *rq;

		rq = list_first_entry(list, struct request, queuelist);
		list_del_init(&rq->queuelist);
		insert_request(hctx, rq, insert_flags, &free);
	}
	spin_unlock_irqrestore(&ad->lock, flags);

	blk_mq_free_requests(&free);
}

// Prepare a request before it is inserted into the scheduler
static void adios_prepare_request(struct request *rq) {
	struct adios_data *ad = rq->q->elevator->elevator_data;
	struct adios_rq_data *rd;

	rq->elv.priv[0] = NULL;

	/* Allocate adios_rq_data from the memory pool */
	rd = kmem_cache_zalloc(ad->rq_data_pool, GFP_ATOMIC);
	if (WARN(!rd, "adios_prepare_request: "
			"Failed to allocate memory from rq_data_pool. rd is NULL\n"))
		return;

	rd->rq = rq;
	rq->elv.priv[0] = rd;
}

static struct adios_rq_data *get_dl_first_rd(struct adios_data *ad, bool idx) {
	struct rb_root_cached *root = &ad->dl_tree[idx];
	struct rb_node *first = rb_first_cached(root);
	struct dl_group *dl_group = rb_entry(first, struct dl_group, node);

	return list_first_entry(&dl_group->rqs, struct adios_rq_data, dl_node);
}

// Select the next request to dispatch from the deadline-sorted red-black tree
static struct request *next_request(struct adios_data *ad) {
	struct adios_rq_data *rd;
	bool dl_idx, bias_idx, reduce_bias;

	if (!ad->dl_queued)
		return NULL;

	dl_idx = ad->dl_queued >> 1;
	rd = get_dl_first_rd(ad, dl_idx);

	bias_idx = ad->dl_bias < 0;
	reduce_bias = (bias_idx == dl_idx);

	if (ad->dl_queued == 0x3) {
		struct adios_rq_data *trd[2];
		trd[0] = get_dl_first_rd(ad, 0);
		trd[1] = rd;

		rd = trd[bias_idx];

		reduce_bias =
			(trd[bias_idx]->deadline > trd[((u8)bias_idx + 1) % 2]->deadline);
	}

	if (reduce_bias) {
		s64 sign = ((int)bias_idx << 1) - 1;
		if (unlikely(!rd->pred_lat))
			ad->dl_bias = sign;
		else {
			ad->dl_bias += sign * (s64)((rd->pred_lat *
				adios_prio_to_weight[ad->dl_prio[bias_idx] + 20]) >> 10);
		}
	}

	return rd->rq;
}

// Reset the batch queue counts for a given page
static void reset_batch_counts(struct adios_data *ad, u8 page) {
	memset(&ad->batch_count[page], 0, sizeof(ad->batch_count[page]));
}

// Initialize all batch queues
static void init_batch_queues(struct adios_data *ad) {
	for (u8 page = 0; page < ADIOS_BQ_PAGES; page++) {
		reset_batch_counts(ad, page);

		for (u8 optype = 0; optype < ADIOS_OPTYPES; optype++)
			INIT_LIST_HEAD(&ad->batch_queue[page][optype]);
	}
}

// Fill the batch queues with requests from the deadline-sorted red-black tree
static bool fill_batch_queues(struct adios_data *ad, u64 current_lat) {
	unsigned long flags;
	u32 count = 0;
	u32 optype_count[ADIOS_OPTYPES] = {0};
	u8 page = (ad->bq_page + 1) % ADIOS_BQ_PAGES;

	reset_batch_counts(ad, page);

	spin_lock_irqsave(&ad->lock, flags);
	while (true) {
		struct request *rq = next_request(ad);
		if (!rq)
			break;

		struct adios_rq_data *rd = get_rq_data(rq);
		u8 optype = adios_optype(rq);
		current_lat += rd->pred_lat;

		// Check batch size and total predicted latency
		if (count && (!ad->latency_model[optype].base || 
			ad->batch_count[page][optype] >= ad->batch_limit[optype] ||
			current_lat > ad->global_latency_window)) {
			break;
		}

		remove_request(ad, rq);

		// Add request to the corresponding batch queue
		list_add_tail(&rq->queuelist, &ad->batch_queue[page][optype]);
		ad->batch_count[page][optype]++;
		atomic64_add(rd->pred_lat, &ad->total_pred_lat);
		optype_count[optype]++;
		count++;
	}
	spin_unlock_irqrestore(&ad->lock, flags);

	if (count) {
		ad->more_bq_ready = true;
		for (u8 optype = 0; optype < ADIOS_OPTYPES; optype++) {
			if (ad->batch_actual_max_size[optype] < optype_count[optype])
				ad->batch_actual_max_size[optype] = optype_count[optype];
		}
		if (ad->batch_actual_max_total < count)
			ad->batch_actual_max_total = count;
	}
	return count;
}

// Flip to the next batch queue page
static void flip_bq_page(struct adios_data *ad) {
	ad->more_bq_ready = false;
	ad->bq_page = (ad->bq_page + 1) % ADIOS_BQ_PAGES;
}

// Dispatch a request from the batch queues
static struct request *dispatch_from_bq(struct adios_data *ad) {
	struct request *rq = NULL;
	u64 tpl;

	guard(spinlock_irqsave)(&ad->bq_lock);

	tpl = atomic64_read(&ad->total_pred_lat);

	if (!ad->more_bq_ready && (!tpl ||
			tpl < div_u64(ad->global_latency_window * ad->bq_refill_below_ratio, 100) ))
		fill_batch_queues(ad, tpl);

again:
	// Check if there are any requests in the batch queues
	for (u8 i = 0; i < ADIOS_OPTYPES; i++) {
		if (!list_empty(&ad->batch_queue[ad->bq_page][i])) {
			rq = list_first_entry(&ad->batch_queue[ad->bq_page][i],
									struct request, queuelist);
			list_del_init(&rq->queuelist);
			return rq;
		}
	}

	// If there's more batch queue page available, flip to it and retry
	if (ad->more_bq_ready) {
		flip_bq_page(ad);
		goto again;
	}

	return NULL;
}

// Dispatch a request from the priority queue
static struct request *dispatch_from_pq(struct adios_data *ad) {
	struct request *rq = NULL;

	guard(spinlock_irqsave)(&ad->pq_lock);

	if (!list_empty(&ad->prio_queue)) {
		rq = list_first_entry(&ad->prio_queue, struct request, queuelist);
		list_del_init(&rq->queuelist);
	}
	return rq;
}

// Dispatch a request to the hardware queue
static struct request *adios_dispatch_request(struct blk_mq_hw_ctx *hctx) {
	struct adios_data *ad = hctx->queue->elevator->elevator_data;
	struct request *rq;

	rq = dispatch_from_pq(ad);
	if (rq) goto found;
	rq = dispatch_from_bq(ad);
	if (!rq) return NULL;
found:
	rq->rq_flags |= RQF_STARTED;
	return rq;
}

// Timer callback function to periodically update latency models
static void update_timer_callback(struct timer_list *t) {
	struct adios_data *ad = from_timer(ad, t, update_timer);

	for (u8 optype = 0; optype < ADIOS_OPTYPES; optype++)
		latency_model_update(&ad->latency_model[optype]);
}

// Handle the completion of a request
static void adios_completed_request(struct request *rq, u64 now) {
	struct adios_data *ad = rq->q->elevator->elevator_data;
	struct adios_rq_data *rd = get_rq_data(rq);

	atomic64_sub(rd->pred_lat, &ad->total_pred_lat);

	if (!rq->io_start_time_ns || !rd->block_size)
		return;
	u64 latency = now - rq->io_start_time_ns;
	u8 optype = adios_optype(rq);
	latency_model_input(&ad->latency_model[optype],
		rd->block_size, latency, rd->pred_lat);
	timer_reduce(&ad->update_timer, jiffies + msecs_to_jiffies(100));
}

// Clean up after a request is finished
static void adios_finish_request(struct request *rq) {
	struct adios_data *ad = rq->q->elevator->elevator_data;

	if (rq->elv.priv[0]) {
		// Free adios_rq_data back to the memory pool
		kmem_cache_free(ad->rq_data_pool, get_rq_data(rq));
		rq->elv.priv[0] = NULL;
	}
}

static inline bool pq_has_work(struct adios_data *ad) {
	guard(spinlock_irqsave)(&ad->pq_lock);
	return !list_empty(&ad->prio_queue);
}

static inline bool bq_has_work(struct adios_data *ad) {
	guard(spinlock_irqsave)(&ad->bq_lock);

	for (u8 i = 0; i < ADIOS_OPTYPES; i++)
		if (!list_empty(&ad->batch_queue[ad->bq_page][i]))
			return true;

	return ad->more_bq_ready;
}

static inline bool dl_tree_has_work(struct adios_data *ad) {
	guard(spinlock_irqsave)(&ad->lock);
	return ad->dl_queued;
}

// Check if there are any requests available for dispatch
static bool adios_has_work(struct blk_mq_hw_ctx *hctx) {
	struct adios_data *ad = hctx->queue->elevator->elevator_data;

	return pq_has_work(ad) || bq_has_work(ad) || dl_tree_has_work(ad);
}

// Initialize the scheduler-specific data for a hardware queue
static int adios_init_hctx(struct blk_mq_hw_ctx *hctx, unsigned int hctx_idx) {
	adios_depth_updated(hctx);
	return 0;
}

// Initialize the scheduler-specific data when initializing the request queue
static int adios_init_sched(struct request_queue *q, struct elevator_type *e) {
	struct adios_data *ad;
	struct elevator_queue *eq;
	int ret = -ENOMEM;

	eq = elevator_alloc(q, e);
	if (!eq)
		return ret;

	ad = kzalloc_node(sizeof(*ad), GFP_KERNEL, q->node);
	if (!ad)
		goto put_eq;

	// Create a memory pool for adios_rq_data
	ad->rq_data_pool = kmem_cache_create("rq_data_pool",
						sizeof(struct adios_rq_data),
						0, SLAB_HWCACHE_ALIGN, NULL);
	if (!ad->rq_data_pool) {
		pr_err("adios: Failed to create rq_data_pool\n");
		goto free_ad;
	}

	/* Create a memory pool for dl_group */
	ad->dl_group_pool = kmem_cache_create("dl_group_pool",
						sizeof(struct dl_group),
						0, SLAB_HWCACHE_ALIGN, NULL);
	if (!ad->dl_group_pool) {
		pr_err("adios: Failed to create dl_group_pool\n");
		goto destroy_rq_data_pool;
	}

	eq->elevator_data = ad;

	ad->global_latency_window = default_global_latency_window;
	ad->bq_refill_below_ratio = default_bq_refill_below_ratio;

	INIT_LIST_HEAD(&ad->prio_queue);
	for (u8 i = 0; i < 2; i++)
		ad->dl_tree[i] = RB_ROOT_CACHED;
	ad->dl_bias = 0;
	ad->dl_queued = 0x0;
	for (u8 i = 0; i < 2; i++)
		ad->dl_prio[i] = default_dl_prio[i];

	for (u8 i = 0; i < ADIOS_OPTYPES; i++) {
		struct latency_model *model = &ad->latency_model[i];
		spin_lock_init(&model->lock);
		spin_lock_init(&model->buckets_lock);
		memset(model->small_bucket, 0,
			sizeof(model->small_bucket[0]) * LM_LAT_BUCKET_COUNT);
		memset(model->large_bucket, 0,
			sizeof(model->large_bucket[0]) * LM_LAT_BUCKET_COUNT);
		model->last_update_jiffies = jiffies;
		model->lm_shrink_at_kreqs  = default_lm_shrink_at_kreqs;
		model->lm_shrink_at_gbytes = default_lm_shrink_at_gbytes;
		model->lm_shrink_resist    = default_lm_shrink_resist;

		ad->latency_target[i] = default_latency_target[i];
		ad->batch_limit[i] = default_batch_limit[i];
	}
	timer_setup(&ad->update_timer, update_timer_callback, 0);
	init_batch_queues(ad);

	spin_lock_init(&ad->lock);
	spin_lock_init(&ad->pq_lock);
	spin_lock_init(&ad->bq_lock);

	/* We dispatch from request queue wide instead of hw queue */
	blk_queue_flag_set(QUEUE_FLAG_SQ_SCHED, q);

	ad->queue = q;
	blk_stat_enable_accounting(q);

	q->elevator = eq;
	return 0;

destroy_rq_data_pool:
	kmem_cache_destroy(ad->rq_data_pool);
free_ad:
	kfree(ad);
put_eq:
	kobject_put(&eq->kobj);
	return ret;
}

// Clean up and free resources when exiting the scheduler
static void adios_exit_sched(struct elevator_queue *e) {
	struct adios_data *ad = e->elevator_data;

	timer_shutdown_sync(&ad->update_timer);

	WARN_ON_ONCE(!list_empty(&ad->prio_queue));

	if (ad->rq_data_pool)
		kmem_cache_destroy(ad->rq_data_pool);

	if (ad->dl_group_pool)
		kmem_cache_destroy(ad->dl_group_pool);

	blk_stat_disable_accounting(ad->queue);

	kfree(ad);
}

// Define sysfs attributes for read operation latency model
#define SYSFS_OPTYPE_DECL(name, optype)					\
static ssize_t adios_lat_model_##name##_show(				\
		struct elevator_queue *e, char *page) {				\
	struct adios_data *ad = e->elevator_data;				\
	struct latency_model *model = &ad->latency_model[optype];		\
	ssize_t len = 0;						\
	guard(spinlock_irqsave)(&model->lock);				\
	len += sprintf(page,       "base : %llu ns\n", model->base);	\
	len += sprintf(page + len, "slope: %llu ns/KiB\n", model->slope);\
	return len;							\
}									\
static ssize_t adios_lat_target_##name##_store(				\
		struct elevator_queue *e, const char *page, size_t count) {	\
	struct adios_data *ad = e->elevator_data;				\
	unsigned long nsec;						\
	int ret;							\
	ret = kstrtoul(page, 10, &nsec);					\
	if (ret)							\
		return ret;						\
	ad->latency_model[optype].base = 0ULL;				\
	ad->latency_target[optype] = nsec;				\
	return count;							\
}									\
static ssize_t adios_lat_target_##name##_show(				\
		struct elevator_queue *e, char *page) {				\
	struct adios_data *ad = e->elevator_data;				\
	return sprintf(page, "%llu\n", ad->latency_target[optype]);	\
}									\
static ssize_t adios_batch_limit_##name##_store(			\
		struct elevator_queue *e, const char *page, size_t count) {	\
	unsigned long max_batch;					\
	int ret;							\
	ret = kstrtoul(page, 10, &max_batch);				\
	if (ret || max_batch == 0)					\
		return -EINVAL;						\
	struct adios_data *ad = e->elevator_data;				\
	ad->batch_limit[optype] = max_batch;				\
	return count;							\
}									\
static ssize_t adios_batch_limit_##name##_show(				\
		struct elevator_queue *e, char *page) {				\
	struct adios_data *ad = e->elevator_data;				\
	return sprintf(page, "%u\n", ad->batch_limit[optype]);		\
}

SYSFS_OPTYPE_DECL(read, ADIOS_READ);
SYSFS_OPTYPE_DECL(write, ADIOS_WRITE);
SYSFS_OPTYPE_DECL(discard, ADIOS_DISCARD);

// Show the maximum batch size actually achieved for each operation type
static ssize_t adios_batch_actual_max_show(
		struct elevator_queue *e, char *page) {
	struct adios_data *ad = e->elevator_data;
	u32 total_count, read_count, write_count, discard_count;

	total_count = ad->batch_actual_max_total;
	read_count = ad->batch_actual_max_size[ADIOS_READ];
	write_count = ad->batch_actual_max_size[ADIOS_WRITE];
	discard_count = ad->batch_actual_max_size[ADIOS_DISCARD];

	return sprintf(page,
		"Total  : %u\nDiscard: %u\nRead   : %u\nWrite  : %u\n",
		total_count, discard_count, read_count, write_count);
}

// Set the global latency window
static ssize_t adios_global_latency_window_store(
		struct elevator_queue *e, const char *page, size_t count) {
	struct adios_data *ad = e->elevator_data;
	unsigned long nsec;
	int ret;

	ret = kstrtoul(page, 10, &nsec);
	if (ret)
		return ret;

	ad->global_latency_window = nsec;

	return count;
}

// Show the global latency window
static ssize_t adios_global_latency_window_show(
		struct elevator_queue *e, char *page) {
	struct adios_data *ad = e->elevator_data;
	return sprintf(page, "%llu\n", ad->global_latency_window);
}

// Show the bq_refill_below_ratio
static ssize_t adios_bq_refill_below_ratio_show(
		struct elevator_queue *e, char *page) {
	struct adios_data *ad = e->elevator_data;
	return sprintf(page, "%d\n", ad->bq_refill_below_ratio);
}

// Set the bq_refill_below_ratio
static ssize_t adios_bq_refill_below_ratio_store(
		struct elevator_queue *e, const char *page, size_t count) {
	struct adios_data *ad = e->elevator_data;
	int ratio;
	int ret;

	ret = kstrtoint(page, 10, &ratio);
	if (ret || ratio < 0 || ratio > 100)
		return -EINVAL;

	ad->bq_refill_below_ratio = ratio;

	return count;
}

// Show the read priority
static ssize_t adios_read_priority_show(
		struct elevator_queue *e, char *page) {
	struct adios_data *ad = e->elevator_data;
	return sprintf(page, "%d\n", ad->dl_prio[0]);
}

// Set the read priority
static ssize_t adios_read_priority_store(
		struct elevator_queue *e, const char *page, size_t count) {
	struct adios_data *ad = e->elevator_data;
	int prio;
	int ret;

	ret = kstrtoint(page, 10, &prio);
	if (ret || prio < -20 || prio > 19)
		return -EINVAL;

	guard(spinlock_irqsave)(&ad->lock);
	ad->dl_prio[0] = prio;
	ad->dl_bias = 0;

	return count;
}

// Reset batch queue statistics
static ssize_t adios_reset_bq_stats_store(
		struct elevator_queue *e, const char *page, size_t count) {
	struct adios_data *ad = e->elevator_data;
	unsigned long val;
	int ret;

	ret = kstrtoul(page, 10, &val);
	if (ret || val != 1)
		return -EINVAL;

	for (u8 i = 0; i < ADIOS_OPTYPES; i++)
		ad->batch_actual_max_size[i] = 0;

	ad->batch_actual_max_total = 0;

	return count;
}

// Reset the latency model parameters
static ssize_t adios_reset_lat_model_store(
		struct elevator_queue *e, const char *page, size_t count) {
	struct adios_data *ad = e->elevator_data;
	unsigned long val;
	int ret;

	ret = kstrtoul(page, 10, &val);
	if (ret || val != 1)
		return -EINVAL;

	for (u8 i = 0; i < ADIOS_OPTYPES; i++) {
		struct latency_model *model = &ad->latency_model[i];
		unsigned long flags;
		spin_lock_irqsave(&model->lock, flags);
		model->base = 0ULL;
		model->slope = 0ULL;
		model->small_sum_delay = 0ULL;
		model->small_count = 0ULL;
		model->large_sum_delay = 0ULL;
		model->large_sum_bsize = 0ULL;
		spin_unlock_irqrestore(&model->lock, flags);
	}

	return count;
}

// Show the ADIOS version
static ssize_t adios_version_show(struct elevator_queue *e, char *page) {
	return sprintf(page, "%s\n", ADIOS_VERSION);
}

// Define sysfs attributes for dynamic thresholds
#define SHRINK_THRESHOLD_ATTR_RW(name, model_field, min_value, max_value) \
static ssize_t adios_shrink_##name##_store( \
		struct elevator_queue *e, const char *page, size_t count) { \
	struct adios_data *ad = e->elevator_data; \
	unsigned long val; \
	int ret; \
	ret = kstrtoul(page, 10, &val); \
	if (ret || val < min_value || val > max_value) \
		return -EINVAL; \
	for (u8 i = 0; i < ADIOS_OPTYPES; i++) { \
		struct latency_model *model = &ad->latency_model[i]; \
		unsigned long flags; \
		spin_lock_irqsave(&model->lock, flags); \
		model->model_field = val; \
		spin_unlock_irqrestore(&model->lock, flags); \
	} \
	return count; \
} \
static ssize_t adios_shrink_##name##_show( \
		struct elevator_queue *e, char *page) { \
	struct adios_data *ad = e->elevator_data; \
	u32 val = 0; \
	for (u8 i = 0; i < ADIOS_OPTYPES; i++) { \
		struct latency_model *model = &ad->latency_model[i]; \
		unsigned long flags; \
		spin_lock_irqsave(&model->lock, flags); \
		val = model->model_field; \
		spin_unlock_irqrestore(&model->lock, flags); \
	} \
	return sprintf(page, "%u\n", val); \
}

SHRINK_THRESHOLD_ATTR_RW(at_kreqs,  lm_shrink_at_kreqs,  1, 100000)
SHRINK_THRESHOLD_ATTR_RW(at_gbytes, lm_shrink_at_gbytes, 1,   1000)
SHRINK_THRESHOLD_ATTR_RW(resist,    lm_shrink_resist,    1,      3)

// Define sysfs attributes
#define AD_ATTR(name, show_func, store_func) \
	__ATTR(name, 0644, show_func, store_func)
#define AD_ATTR_RW(name) \
	__ATTR(name, 0644, adios_##name##_show, adios_##name##_store)
#define AD_ATTR_RO(name) \
	__ATTR(name, 0644, adios_##name##_show, NULL)
#define AD_ATTR_WO(name) \
	__ATTR(name, 0644, NULL, adios_##name##_store)

// Define sysfs attributes for ADIOS scheduler
static struct elv_fs_entry adios_sched_attrs[] = {
	AD_ATTR_RO(batch_actual_max),
	AD_ATTR_RW(bq_refill_below_ratio),
	AD_ATTR_RW(global_latency_window),

	AD_ATTR_RW(batch_limit_read),
	AD_ATTR_RW(batch_limit_write),
	AD_ATTR_RW(batch_limit_discard),

	AD_ATTR_RO(lat_model_read),
	AD_ATTR_RO(lat_model_write),
	AD_ATTR_RO(lat_model_discard),

	AD_ATTR_RW(lat_target_read),
	AD_ATTR_RW(lat_target_write),
	AD_ATTR_RW(lat_target_discard),

	AD_ATTR_RW(shrink_at_kreqs),
	AD_ATTR_RW(shrink_at_gbytes),
	AD_ATTR_RW(shrink_resist),

	AD_ATTR_RW(read_priority),

	AD_ATTR_WO(reset_bq_stats),
	AD_ATTR_WO(reset_lat_model),
	AD_ATTR(adios_version, adios_version_show, NULL),

	__ATTR_NULL
};

// Define the ADIOS scheduler type
static struct elevator_type mq_adios = {
	.ops = {
		.next_request		= elv_rb_latter_request,
		.former_request		= elv_rb_former_request,
		.limit_depth		= adios_limit_depth,
		.depth_updated		= adios_depth_updated,
		.request_merged		= adios_request_merged,
		.requests_merged	= adios_merged_requests,
		.bio_merge			= adios_bio_merge,
		.insert_requests	= adios_insert_requests,
		.prepare_request	= adios_prepare_request,
		.dispatch_request	= adios_dispatch_request,
		.completed_request	= adios_completed_request,
		.finish_request		= adios_finish_request,
		.has_work			= adios_has_work,
		.init_hctx			= adios_init_hctx,
		.init_sched			= adios_init_sched,
		.exit_sched			= adios_exit_sched,
	},
	.elevator_attrs = adios_sched_attrs,
	.elevator_name = "adios",
	.elevator_owner = THIS_MODULE,
};
MODULE_ALIAS("mq-adios-iosched");

#define ADIOS_PROGNAME "Adaptive Deadline I/O Scheduler"
#define ADIOS_AUTHOR   "Masahito Suzuki"

// Initialize the ADIOS scheduler module
static int __init adios_init(void) {
	printk(KERN_INFO "%s %s by %s\n",
		ADIOS_PROGNAME, ADIOS_VERSION, ADIOS_AUTHOR);
	return elv_register(&mq_adios);
}

// Exit the ADIOS scheduler module
static void __exit adios_exit(void) {
	elv_unregister(&mq_adios);
}

module_init(adios_init);
module_exit(adios_exit);

MODULE_AUTHOR(ADIOS_AUTHOR);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(ADIOS_PROGNAME);