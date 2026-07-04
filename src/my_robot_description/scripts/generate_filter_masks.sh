#!/usr/bin/env bash
# generate_filter_masks.sh — regenerate keepout_mask.pgm to match your saved map.
#
# Run AFTER saving a map with save_map.sh:
#   ~/ros2_ws/src/my_robot_description/scripts/generate_filter_masks.sh ~/maps/my_map.yaml
#
# Output:
#   ~/ros2_ws/src/my_robot_description/maps/keepout_mask.pgm  (all-free — edit in GIMP to add keepout zones)
#   ~/ros2_ws/src/my_robot_description/maps/keepout_mask.yaml (matching metadata)

set -euo pipefail

MAP_YAML="${1:-}"
if [[ -z "$MAP_YAML" ]]; then
  echo "Usage: $0 <path/to/map.yaml>"
  exit 1
fi

MASK_DIR="$(dirname "$(realpath "$0")")/../maps"

# Parse map dimensions from YAML
RES=$(grep 'resolution:' "$MAP_YAML" | awk '{print $2}')
ORIGIN_X=$(grep 'origin:' "$MAP_YAML" | sed 's/.*\[//;s/,.*//')
ORIGIN_Y=$(grep 'origin:' "$MAP_YAML" | sed 's/.*\[//;s/[^,]*,//;s/,.*//')
MAP_PGM="$(dirname "$MAP_YAML")/$(grep 'image:' "$MAP_YAML" | awk '{print $2}')"

# Get pixel dimensions from PGM header
read -r _ W H _ < <(head -3 "$MAP_PGM" | tr '\n' ' ')
echo "Map: ${W}x${H} px, res=${RES} m/px, origin=(${ORIGIN_X}, ${ORIGIN_Y})"

# Write keepout_mask.yaml
cat > "$MASK_DIR/keepout_mask.yaml" << YAML
image: keepout_mask.pgm
resolution: $RES
origin: [$ORIGIN_X, $ORIGIN_Y, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
YAML

# Write all-free PGM (binary P5)
python3 - "$W" "$H" "$MASK_DIR/keepout_mask.pgm" << 'PY'
import sys
w, h = int(sys.argv[1]), int(sys.argv[2])
out  = sys.argv[3]
with open(out, 'wb') as f:
    f.write(f"P5\n{w} {h}\n255\n".encode())
    f.write(bytes([254] * w * h))
PY

echo ""
echo "Generated: $MASK_DIR/keepout_mask.yaml"
echo "Generated: $MASK_DIR/keepout_mask.pgm  (${W}x${H}, all free)"
echo ""
echo "To add keepout zones:"
echo "  1. Open $MASK_DIR/keepout_mask.pgm in GIMP"
echo "  2. Paint black (value=0) over restricted zones"
echo "  3. Export as PGM (keep same dimensions)"
echo "  4. Launch navigation with: use_keepout_filter:=true"
