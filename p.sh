#!/bin/bash

# Linux Kernel Patch Script
# Supports single patch URL or batch patching with sorting
# Usage: ./patch_kernel.sh [OPTIONS] [PATCH_URL or PATCH_DIRECTORY]

set -e

# Configuration
KERNEL_DIR="${KERNEL_DIR:-$(pwd)}"
PATCH_DIR="${PATCH_DIR:-./patches}"
DRY_RUN=false
VERBOSE=false
REVERSE=false
BACKUP=true
STRIP_LEVEL=1

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging function
log() {
    echo -e "${BLUE}[$(date +'%Y-%m-%d %H:%M:%S')]${NC} $1"
}

error() {
    echo -e "${RED}[ERROR]${NC} $1" >&2
}

success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

warn() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# Help function
show_help() {
    cat << EOF
Linux Kernel Patch Script

USAGE:
    $0 [OPTIONS] [PATCH_URL or PATCH_DIRECTORY]

OPTIONS:
    -h, --help          Show this help message
    -d, --dry-run       Show what would be done without applying patches
    -v, --verbose       Enable verbose output
    -r, --reverse       Reverse patches instead of applying them
    -b, --no-backup     Don't create backup before patching
    -p, --strip LEVEL   Strip LEVEL leading directories (default: 1)
    -k, --kernel-dir    Kernel source directory (default: current dir)
    --patch-dir         Directory to store downloaded patches (default: ./patches)

EXAMPLES:
    # Apply single patch from URL
    $0 https://lkml.org/lkml/2024/1/1/1.patch

    # Batch patch from directory (sorted)
    $0 /path/to/patches/

    # Dry run to see what would be applied
    $0 --dry-run /path/to/patches/

    # Reverse patches
    $0 --reverse /path/to/patches/

ENVIRONMENT VARIABLES:
    KERNEL_DIR     - Kernel source directory
    PATCH_DIR      - Directory for downloaded patches
EOF
}

# Check if we're in a kernel source directory
check_kernel_dir() {
    if [[ ! -f "$KERNEL_DIR/Makefile" ]] || ! grep -q "^VERSION\|^PATCHLEVEL" "$KERNEL_DIR/Makefile" 2>/dev/null; then
        error "Not in a Linux kernel source directory: $KERNEL_DIR"
        error "Please run from kernel source root or set KERNEL_DIR"
        exit 1
    fi
    
    if [[ $VERBOSE == true ]]; then
        local version=$(grep "^VERSION" "$KERNEL_DIR/Makefile" | cut -d'=' -f2 | tr -d ' ')
        local patchlevel=$(grep "^PATCHLEVEL" "$KERNEL_DIR/Makefile" | cut -d'=' -f2 | tr -d ' ')
        log "Found kernel source v$version.$patchlevel in $KERNEL_DIR"
    fi
}

# Download patch from URL
download_patch() {
    local url="$1"
    local filename="$2"
    
    log "Downloading patch from: $url"
    
    if command -v curl >/dev/null 2>&1; then
        curl -sSL "$url" -o "$filename"
    elif command -v wget >/dev/null 2>&1; then
        wget -q "$url" -O "$filename"
    else
        error "Neither curl nor wget found. Please install one of them."
        exit 1
    fi
    
    if [[ ! -s "$filename" ]]; then
        error "Failed to download patch or file is empty"
        exit 1
    fi
    
    success "Downloaded patch to: $filename"
}

# Sort patches numerically/alphabetically
sort_patches() {
    local patch_dir="$1"
    
    # Find all patch files and sort them
    find "$patch_dir" -name "*.patch" -o -name "*.diff" | sort -V
}

# Validate patch file
validate_patch() {
    local patch_file="$1"
    
    if [[ ! -f "$patch_file" ]]; then
        error "Patch file not found: $patch_file"
        return 1
    fi
    
    if [[ ! -s "$patch_file" ]]; then
        error "Patch file is empty: $patch_file"
        return 1
    fi
    
    # Basic validation - check if it looks like a patch
    if ! grep -q "^[+-]" "$patch_file" && ! grep -q "^@@" "$patch_file"; then
        warn "File may not be a valid patch: $patch_file"
        return 1
    fi
    
    return 0
}

# Create backup
create_backup() {
    local backup_dir="$KERNEL_DIR/.patch_backup_$(date +%Y%m%d_%H%M%S)"
    
    if [[ $BACKUP == true ]]; then
        log "Creating backup in: $backup_dir"
        mkdir -p "$backup_dir"
        
        # Create a git stash or simple backup
        if git -C "$KERNEL_DIR" rev-parse --git-dir >/dev/null 2>&1; then
            git -C "$KERNEL_DIR" stash push -m "Pre-patch backup $(date)"
            echo "$backup_dir" > "$KERNEL_DIR/.last_patch_backup"
        else
            # Simple file backup for modified files
            find "$KERNEL_DIR" -name "*.orig" -exec cp {} "$backup_dir/" \; 2>/dev/null || true
        fi
    fi
}

# Apply single patch
apply_patch() {
    local patch_file="$1"
    local patch_name=$(basename "$patch_file")
    
    log "Processing patch: $patch_name"
    
    if ! validate_patch "$patch_file"; then
        return 1
    fi
    
    # Check if patch can be applied
    local patch_cmd="patch -p$STRIP_LEVEL"
    if [[ $REVERSE == true ]]; then
        patch_cmd="$patch_cmd -R"
    fi
    
    if [[ $DRY_RUN == true ]]; then
        log "DRY RUN: Would apply patch with: $patch_cmd --dry-run < $patch_file"
        if $patch_cmd --dry-run -s < "$patch_file" 2>/dev/null; then
            success "Patch can be applied: $patch_name"
        else
            error "Patch cannot be applied: $patch_name"
            return 1
        fi
        return 0
    fi
    
    # Apply the patch
    log "Applying patch: $patch_name"
    if [[ $VERBOSE == true ]]; then
        $patch_cmd < "$patch_file"
    else
        $patch_cmd -s < "$patch_file"
    fi
    
    if [[ $? -eq 0 ]]; then
        success "Successfully applied: $patch_name"
        return 0
    else
        error "Failed to apply patch: $patch_name"
        return 1
    fi
}

# Main function
main() {
    local input="$1"
    local failed_patches=()
    local applied_patches=()
    
    check_kernel_dir
    
    # Change to kernel directory
    cd "$KERNEL_DIR"
    
    # Handle single URL
    if [[ "$input" =~ ^https?:// ]]; then
        mkdir -p "$PATCH_DIR"
        local patch_filename="$PATCH_DIR/$(basename "$input" .patch).patch"
        
        download_patch "$input" "$patch_filename"
        
        create_backup
        
        if apply_patch "$patch_filename"; then
            applied_patches+=("$patch_filename")
        else
            failed_patches+=("$patch_filename")
        fi
        
    # Handle directory (batch processing)
    elif [[ -d "$input" ]]; then
        log "Batch processing patches from: $input"
        
        # Get sorted list of patches
        local patches=($(sort_patches "$input"))
        
        if [[ ${#patches[@]} -eq 0 ]]; then
            error "No patch files found in: $input"
            exit 1
        fi
        
        log "Found ${#patches[@]} patch files"
        
        create_backup
        
        # Process each patch in order
        for patch_file in "${patches[@]}"; do
            if apply_patch "$patch_file"; then
                applied_patches+=("$patch_file")
            else
                failed_patches+=("$patch_file")
                
                # Ask if user wants to continue
                if [[ $DRY_RUN == false ]]; then
                    read -p "Continue with remaining patches? (y/N): " -n 1 -r
                    echo
                    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                        break
                    fi
                fi
            fi
        done
        
    # Handle single file
    elif [[ -f "$input" ]]; then
        create_backup
        
        if apply_patch "$input"; then
            applied_patches+=("$input")
        else
            failed_patches+=("$input")
        fi
        
    else
        error "Invalid input: $input"
        error "Please provide a URL, directory, or patch file"
        exit 1
    fi
    
    # Summary
    echo
    log "=== PATCH SUMMARY ==="
    log "Applied: ${#applied_patches[@]} patches"
    log "Failed:  ${#failed_patches[@]} patches"
    
    if [[ ${#applied_patches[@]} -gt 0 ]]; then
        success "Successfully applied patches:"
        for patch in "${applied_patches[@]}"; do
            echo "  - $(basename "$patch")"
        done
    fi
    
    if [[ ${#failed_patches[@]} -gt 0 ]]; then
        error "Failed patches:"
        for patch in "${failed_patches[@]}"; do
            echo "  - $(basename "$patch")"
        done
        exit 1
    fi
    
    success "All patches processed successfully!"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        -d|--dry-run)
            DRY_RUN=true
            shift
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -r|--reverse)
            REVERSE=true
            shift
            ;;
        -b|--no-backup)
            BACKUP=false
            shift
            ;;
        -p|--strip)
            STRIP_LEVEL="$2"
            shift 2
            ;;
        -k|--kernel-dir)
            KERNEL_DIR="$2"
            shift 2
            ;;
        --patch-dir)
            PATCH_DIR="$2"
            shift 2
            ;;
        -*)
            error "Unknown option: $1"
            show_help
            exit 1
            ;;
        *)
            # This is the input (URL, directory, or file)
            if [[ -z "$INPUT" ]]; then
                INPUT="$1"
            else
                error "Multiple inputs provided. Please specify only one."
                exit 1
            fi
            shift
            ;;
    esac
done

# Check if input was provided
if [[ -z "$INPUT" ]]; then
    error "No input provided. Please specify a patch URL, directory, or file."
    show_help
    exit 1
fi

# Run main function
main "$INPUT"