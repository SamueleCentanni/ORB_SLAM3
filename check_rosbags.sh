#!/usr/bin/env bash
set -euo pipefail

# Usage: ./check_rosbags.sh /path/to/dataset_root
# Finds ROS2 bag folders (containing metadata.yaml and *.db3) under dataset_root
# and generates ORB-SLAM3 EuRoC-style datasets in a sibling folder named "dataset".
# Example structure:
#   <run_dir>/rosbag/metadata.yaml + *.db3
#   -> outputs to <run_dir>/dataset/

DRY_RUN=false
ROOT_DIR=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry_run)
      DRY_RUN=true
      shift
      ;;
    -h|--help)
      echo "Usage: $0 [--dry_run] /path/to/dataset_root"
      exit 0
      ;;
    *)
      if [[ -z "$ROOT_DIR" ]]; then
        ROOT_DIR="$1"
        shift
      else
        echo "Error: unexpected argument: $1"
        echo "Usage: $0 [--dry_run] /path/to/dataset_root"
        exit 1
      fi
      ;;
  esac
done

if [[ -z "$ROOT_DIR" ]]; then
  echo "Usage: $0 [--dry_run] /path/to/dataset_root"
  exit 1
fi
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXTRACTOR="${SCRIPT_DIR}/extract_rosbag2.py"

if [[ ! -f "$EXTRACTOR" ]]; then
  echo "Error: extractor not found at $EXTRACTOR"
  exit 1
fi

if [[ ! -d "$ROOT_DIR" ]]; then
  echo "Error: root directory not found: $ROOT_DIR"
  exit 1
fi

# Find rosbag directories by metadata.yaml
mapfile -t BAG_DIRS < <(find "$ROOT_DIR" -type f -name metadata.yaml -printf '%h\n' | sort -u)

if [[ ${#BAG_DIRS[@]} -eq 0 ]]; then
  echo "No rosbag directories found under: $ROOT_DIR"
  exit 0
fi

for BAG_DIR in "${BAG_DIRS[@]}"; do
  # Only consider dirs that have at least one .db3 file
  if ! ls "$BAG_DIR"/*.db3 >/dev/null 2>&1; then
    echo "Skipping (no .db3 found): $BAG_DIR"
    continue
  fi

  # Default output: sibling folder named 'dataset' in the same parent as rosbag
  RUN_DIR="$(dirname "$BAG_DIR")"
  OUTPUT_DIR="${RUN_DIR}/dataset"

  if [[ -d "$OUTPUT_DIR" ]] && { [[ -f "$OUTPUT_DIR/timestamps.txt" ]] || [[ -d "$OUTPUT_DIR/mav0" ]]; }; then
    echo "Skipping (dataset exists): $OUTPUT_DIR"
    continue
  fi

  echo "============================================================"
  echo "ROSBAG:  $BAG_DIR"
  echo "OUTPUT:  $OUTPUT_DIR"
  echo "------------------------------------------------------------"

  if [[ "$DRY_RUN" == true ]]; then
    echo "DRY RUN: would extract bag '$BAG_DIR' to '$OUTPUT_DIR'"
  else
    python3 "$EXTRACTOR" \
      --bag "$BAG_DIR" \
      --output "$OUTPUT_DIR"
  fi

done

echo "Done."