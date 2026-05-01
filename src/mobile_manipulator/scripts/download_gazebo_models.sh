#!/bin/bash
# Downloads textured Gazebo Classic models that YOLOv8 (COCO-trained) can reliably detect.
#
# Sources:
#   osrf/gazebo_models  — official models with meshes + textures (coke_can etc.)
#   Gazebo Fuel REST    — community models (apple, orange, banana with real textures)
#
# Run from anywhere; models go into the gazebo_models directory next to this script.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MODELS_DIR="$SCRIPT_DIR/../gazebo_models"
TMP=$(mktemp -d)
trap "rm -rf $TMP" EXIT

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }

# ── Helper: sparse-clone one model from osrf/gazebo_models ────────────────────
clone_osrf() {
    local model_name=$1
    local target="$MODELS_DIR/$model_name"
    if [ -d "$target" ]; then
        info "$model_name already exists, skipping."
        return 0
    fi
    info "Downloading $model_name from osrf/gazebo_models..."
    git clone --no-checkout --depth=1 --filter=blob:none \
        https://github.com/osrf/gazebo_models.git "$TMP/osrf_$model_name" 2>/dev/null
    cd "$TMP/osrf_$model_name"
    git sparse-checkout set "$model_name"
    git checkout 2>/dev/null
    if [ -d "$model_name" ]; then
        cp -r "$model_name" "$target"
        info "  → Installed to $target"
    else
        error "  $model_name not found in osrf/gazebo_models."
    fi
    cd /
}

# ── Helper: download one model zip from Gazebo Fuel ───────────────────────────
# Fuel zip layout: model.config + model.sdf + meshes/ + materials/ at root.
download_fuel() {
    local owner=$1
    local model_name=$2
    local coco_name=$3          # COCO-80 class label (for reference)
    local target="$MODELS_DIR/$model_name"

    if [ -d "$target" ]; then
        info "$model_name already exists, skipping."
        return 0
    fi
    info "Downloading $model_name (COCO: $coco_name) from Gazebo Fuel..."
    local url="https://fuel.gazebosim.org/1.0/${owner}/models/${model_name}/zip"
    if wget -q --show-progress -O "$TMP/${model_name}.zip" "$url" 2>/dev/null; then
        mkdir -p "$target"
        unzip -q "$TMP/${model_name}.zip" -d "$target"
        # Fuel zips sometimes add a nested directory — flatten if needed
        inner=$(find "$target" -maxdepth 1 -mindepth 1 -type d | head -1)
        if [ -n "$inner" ] && [ ! -f "$target/model.sdf" ]; then
            mv "$inner"/* "$target/"
            rmdir "$inner" 2>/dev/null || true
        fi
        info "  → Installed to $target"
    else
        warn "  Could not download $model_name from Fuel (no internet or model moved)."
        warn "  Browse manually: https://fuel.gazebosim.org/1.0/${owner}/models/${model_name}"
    fi
}

# ── Models to download ─────────────────────────────────────────────────────────
#
# osrf/gazebo_models — well-established, textured, mesh-based:
#   coke_can  → COCO "bottle"  (Coca-Cola can with proper cylindrical mesh + texture)
#
# Gazebo Fuel — community models with realistic appearance:
#   Apple     → COCO "apple"   (textured apple mesh)
#   Banana    → COCO "banana"  (textured banana mesh)
#   Orange    → COCO "orange"  (textured orange mesh)
#   Wine Glass → COCO "wine glass"

clone_osrf "coke_can"

download_fuel "OpenRobotics" "Apple"     "apple"
download_fuel "OpenRobotics" "Banana"    "banana"
download_fuel "OpenRobotics" "Orange"    "orange"

echo ""
info "Done. Next steps:"
echo "  1. Add new models to small_house.world (see comments below)"
echo "  2. Add entries to semantic_map.yaml under the relevant region's 'objects' list"
echo "  3. catkin_make && relaunch Gazebo"
echo ""
echo "World file snippet for coke_can (COCO: bottle) — place near coffee_table:"
cat <<'EOF'
    <model name='graspable_coke'>
        <include><uri>model://coke_can</uri></include>
        <pose frame=''>1.50 -1.90 0.50 0 0 0</pose>
    </model>
EOF
echo ""
echo "World file snippet for Apple (COCO: apple) — place on kitchen_table:"
cat <<'EOF'
    <model name='graspable_apple2'>
        <include><uri>model://Apple</uri></include>
        <pose frame=''>6.55 0.80 0.90 0 0 0</pose>
    </model>
EOF
