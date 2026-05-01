#!/usr/bin/env python3
"""
Transforms semantic_map.yaml from one map's coordinate frame to another
using image cross-correlation to automatically compute the 2D translation
between the two OccupancyGrid maps.

Usage:
  python3 remap_semantic_map.py \
      --old_map  maps/small_house_map.yaml \
      --new_map  maps/toolboxMap.yaml \
      --semantic config/semantic_map.yaml \
      --output   config/semantic_map.yaml

The script:
  1. Renders both maps in a shared pixel space at a common resolution.
  2. Finds the (dx_world, dy_world) translation via FFT phase correlation
     on the occupied-cell binary images.
  3. Applies the translation (and optional manual rotation) to every
     (x, y) centre and every polygon vertex in the semantic map.
"""

import argparse, os, math
import numpy as np
import yaml
from PIL import Image


# ── map loading ──────────────────────────────────────────────────────────────

def load_map(yaml_path):
    with open(yaml_path) as f:
        info = yaml.safe_load(f)
    img_path = info['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)
    img = np.array(Image.open(img_path).convert('L'))  # H x W, uint8
    res  = float(info['resolution'])
    ox   = float(info['origin'][0])
    oy   = float(info['origin'][1])
    return img, res, ox, oy


def to_binary_occupied(img):
    """Return float32 array: 1 where occupied (dark pixels), 0 elsewhere."""
    out = np.zeros(img.shape, dtype=np.float32)
    out[img < 50] = 1.0   # occupied cells are near-black in standard PGM maps
    return out


# ── phase correlation translation estimator ─────────────────────────────────

def phase_correlation(a, b):
    """
    Estimate the integer pixel shift (dy, dx) that best aligns image b onto a.
    Returns (shift_row, shift_col) in image-pixel units.
    """
    FA = np.fft.fft2(a)
    FB = np.fft.fft2(b)
    denom = np.abs(FA * np.conj(FB))
    denom[denom == 0] = 1e-10
    R = (FA * np.conj(FB)) / denom
    r = np.fft.ifft2(R).real
    shift = np.unravel_index(np.argmax(r), r.shape)
    # Convert wrap-around shift to signed offset
    rows, cols = a.shape
    sr = shift[0] if shift[0] < rows / 2 else shift[0] - rows
    sc = shift[1] if shift[1] < cols / 2 else shift[1] - cols
    return sr, sc


def find_world_translation(old_img, old_res, old_ox, old_oy,
                            new_img, new_res, new_ox, new_oy):
    """
    Render both maps into a common pixel grid and run phase correlation.
    Returns (dx_world, dy_world): amount to ADD to old-map coordinates to get
    new-map coordinates.
    """
    assert abs(old_res - new_res) < 1e-6, \
        f"Resolutions differ: {old_res} vs {new_res}. Resample first."
    res = old_res

    # World bounding box that covers both maps
    oh, ow = old_img.shape
    nh, nw = new_img.shape

    old_wx_max = old_ox + ow * res
    old_wy_max = old_oy + oh * res
    new_wx_max = new_ox + nw * res
    new_wy_max = new_oy + nh * res

    wx_min = min(old_ox, new_ox)
    wy_min = min(old_oy, new_oy)
    wx_max = max(old_wx_max, new_wx_max)
    wy_max = max(old_wy_max, new_wy_max)

    canvas_w = int(math.ceil((wx_max - wx_min) / res))
    canvas_h = int(math.ceil((wy_max - wy_min) / res))

    def paste_onto_canvas(img, ox, oy):
        canvas = np.zeros((canvas_h, canvas_w), dtype=np.float32)
        col0 = int(round((ox - wx_min) / res))
        row0 = canvas_h - int(round((oy - wy_min) / res)) - img.shape[0]
        h, w = img.shape
        # Clamp to canvas
        r0 = max(row0, 0); r1 = min(row0 + h, canvas_h)
        c0 = max(col0, 0); c1 = min(col0 + w, canvas_w)
        # Source slice
        sr0 = r0 - row0; sr1 = sr0 + (r1 - r0)
        sc0 = c0 - col0; sc1 = sc0 + (c1 - c0)
        src = to_binary_occupied(img)
        # PGM row-0 = top of image = wy_max end; flip vertically
        src = np.flipud(src)
        canvas[r0:r1, c0:c1] = src[sr0:sr1, sc0:sc1]
        return canvas

    ca = paste_onto_canvas(old_img, old_ox, old_oy)
    cb = paste_onto_canvas(new_img, new_ox, new_oy)

    sr, sc = phase_correlation(ca, cb)
    print(f"  Phase correlation shift: rows={sr:+d}  cols={sc:+d} pixels")

    dx_world = sc * res   # positive col shift → positive x
    dy_world = sr * res   # positive row shift → positive y (canvas rows go up)
    return dx_world, dy_world


# ── coordinate transform ─────────────────────────────────────────────────────

def f(v):
    """Convert any numpy scalar to a plain Python float so yaml.dump
    writes e.g. '1.23' instead of a numpy object tag."""
    return float(v)


def transform_point(x, y, dx, dy, dtheta=0.0):
    cos_t, sin_t = math.cos(dtheta), math.sin(dtheta)
    xr = cos_t * x - sin_t * y + dx
    yr = sin_t * x + cos_t * y + dy
    return f(xr), f(yr)


def transform_semantic(semantic, dx, dy, dtheta=0.0):
    for region in semantic.get('regions', []):
        x, y = region['x'], region['y']
        region['x'], region['y'] = transform_point(x, y, dx, dy, dtheta)

        region['yaw'] = f(region.get('yaw', 0.0) + dtheta)

        region['polygon'] = [
            list(transform_point(vx, vy, dx, dy, dtheta))
            for vx, vy in region.get('polygon', [])
        ]
    return semantic


# ── main ─────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--old_map',  required=True, help='Path to old map YAML (e.g. small_house_map.yaml)')
    p.add_argument('--new_map',  required=True, help='Path to new map YAML (e.g. toolboxMap.yaml)')
    p.add_argument('--semantic', required=True, help='Path to semantic_map.yaml to transform')
    p.add_argument('--output',   required=True, help='Output path for corrected semantic_map.yaml')
    p.add_argument('--dx',  type=float, default=None,
                   help='Override: manual translation x (world metres)')
    p.add_argument('--dy',  type=float, default=None,
                   help='Override: manual translation y (world metres)')
    p.add_argument('--dtheta', type=float, default=0.0,
                   help='Override: rotation offset in radians (default 0)')
    args = p.parse_args()

    print("Loading maps …")
    old_img, old_res, old_ox, old_oy = load_map(args.old_map)
    new_img, new_res, new_ox, new_oy = load_map(args.new_map)
    print(f"  old map: {old_img.shape}  origin=({old_ox:.3f}, {old_oy:.3f})  res={old_res}")
    print(f"  new map: {new_img.shape}  origin=({new_ox:.3f}, {new_oy:.3f})  res={new_res}")

    if args.dx is not None and args.dy is not None:
        dx, dy = args.dx, args.dy
        print(f"Using manual translation: dx={dx:.3f}  dy={dy:.3f}")
    else:
        print("Computing translation via phase correlation …")
        dx, dy = find_world_translation(
            old_img, old_res, old_ox, old_oy,
            new_img, new_res, new_ox, new_oy)
        print(f"  Estimated world translation: dx={dx:.3f} m  dy={dy:.3f} m")

    print(f"Applying transform (dx={dx:.3f}, dy={dy:.3f}, dtheta={args.dtheta:.4f}) …")
    with open(args.semantic) as f:
        semantic = yaml.safe_load(f)

    semantic = transform_semantic(semantic, dx, dy, args.dtheta)

    with open(args.output, 'w') as f:
        yaml.dump(semantic, f, default_flow_style=False, allow_unicode=True)
    print(f"Saved → {args.output}")


if __name__ == '__main__':
    main()
