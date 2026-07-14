#!/usr/bin/env python3
"""Validate YOLO mine masks, semantic visibility and scene-level split isolation."""

import argparse
import hashlib
import json
import random
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np


def parse_label(path):
    instances = []
    for lineno, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not raw.strip():
            continue
        parts = raw.split()
        if len(parts) < 7 or (len(parts) - 1) % 2:
            raise ValueError(f"{path}:{lineno}: expected class plus >=3 xy pairs")
        if int(parts[0]) != 0:
            raise ValueError(f"{path}:{lineno}: only class 0 is allowed")
        xy = np.asarray([float(v) for v in parts[1:]], np.float64).reshape(-1, 2)
        if not np.isfinite(xy).all() or (xy < 0).any() or (xy > 1).any():
            raise ValueError(f"{path}:{lineno}: polygon coordinates outside [0,1]")
        instances.append(xy)
    return instances


def average_hash(image):
    gray = cv2.resize(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (8, 8), interpolation=cv2.INTER_AREA)
    return (gray > np.mean(gray)).reshape(-1)


def visible_contrast(image, polygon_px):
    """Mean per-channel deviation of labelled pixels from a local background ring."""
    height, width = image.shape[:2]
    mask = np.zeros((height, width), np.uint8)
    cv2.fillPoly(mask, [np.round(polygon_px).astype(np.int32)], 1)
    outer = cv2.dilate(mask, np.ones((9, 9), np.uint8), iterations=1) - mask
    inside_pixels = image[mask.astype(bool)].astype(np.float32)
    outside_pixels = image[outer.astype(bool)].astype(np.float32)
    if not len(inside_pixels) or not len(outside_pixels):
        return float("inf")
    background = np.median(outside_pixels, axis=0)
    return float(np.mean(np.abs(inside_pixels - background)))


def scene_group(metadata):
    """Return the smallest unit that must never be split across train/val/test."""
    domain = metadata.get("source_domain") or metadata.get("domain_tag") or "unknown"
    cycle = int(metadata.get("domain_cycle", -1))
    seed = int(metadata.get("randomizer_seed", -1))
    session = str(metadata.get("session", "unknown"))
    if seed >= 0:
        run = f"seed_{seed}"
    elif session.startswith("curated_"):
        # Legacy curation renamed 500-frame blocks even when one randomizer
        # scene crossed several blocks. Domain+cycle reconstructs that scene.
        run = f"legacy_{domain}"
    else:
        run = session
    return f"{domain}|{run}|cycle_{cycle}"


def load_metadata(root):
    result = {}
    for split in ("train", "val", "test"):
        path = root / "metadata" / f"{split}.jsonl"
        if not path.is_file():
            continue
        for lineno, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if not raw.strip():
                continue
            item = json.loads(raw)
            image = item.get("image")
            if not image:
                raise ValueError(f"{path}:{lineno}: metadata has no image field")
            result[image] = item
    return result


def percentile_summary(values):
    if not values:
        return {"count": 0}
    array = np.asarray(values, np.float64)
    return {"count": int(array.size), "min": float(array.min()),
            "p01": float(np.percentile(array, 1)), "p05": float(np.percentile(array, 5)),
            "median": float(np.median(array)), "p95": float(np.percentile(array, 95)),
            "max": float(array.max())}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset_dir")
    parser.add_argument("--previews", type=int, default=24)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--min-visible-contrast", type=float, default=3.0,
                        help="flag masks whose pixels barely differ from their local background; <=0 disables")
    parser.add_argument("--fail-on-low-contrast", action="store_true",
                        help="treat any low-contrast mask as an audit error")
    parser.add_argument("--fail-on-scene-leakage", action="store_true",
                        help="fail when one randomizer seed+cycle appears in multiple splits")
    parser.add_argument("--require-metadata", action="store_true",
                        help="require metadata for every image so scene leakage can be checked")
    parser.add_argument("--require-training-ready", action="store_true",
                        help="require every split to contain both positive and negative images")
    args = parser.parse_args()

    root = Path(args.dataset_dir).expanduser().resolve()
    errors, warnings = [], []
    stats = {"dataset": str(root), "splits": {}, "errors": errors, "warnings": warnings}
    seen_sha = {}
    sessions = defaultdict(set)
    scene_splits = defaultdict(set)
    preview_candidates = []
    low_contrast_records = []
    try:
        metadata = load_metadata(root)
    except Exception as exc:
        metadata = {}
        errors.append(f"metadata parse error: {exc}")

    for split in ("train", "val", "test"):
        image_dir, label_dir = root / "images" / split, root / "labels" / split
        if not image_dir.is_dir() or not label_dir.is_dir():
            errors.append(f"missing split directories for {split}")
            continue
        images = {path.stem: path for path in image_dir.glob("*.jpg")}
        labels = {path.stem: path for path in label_dir.glob("*.txt")}
        for stem in sorted(images.keys() - labels.keys()):
            errors.append(f"missing label: {split}/{stem}")
        for stem in sorted(labels.keys() - images.keys()):
            errors.append(f"orphan label: {split}/{stem}")

        positives = negatives = instances_count = 0
        areas, contrasts, recent_hashes = [], [], []
        split_low_images = set()
        for stem in sorted(images.keys() & labels.keys()):
            image_path = images[stem]
            image = cv2.imread(str(image_path))
            if image is None:
                errors.append(f"unreadable image: {image_path}")
                continue
            try:
                instances = parse_label(labels[stem])
            except Exception as exc:
                errors.append(str(exc))
                continue
            positives += int(bool(instances)); negatives += int(not instances)
            instances_count += len(instances)
            height, width = image.shape[:2]
            image_min_contrast = float("inf")
            for instance_index, polygon in enumerate(instances):
                polygon_px = polygon * [width, height]
                area = abs(cv2.contourArea(polygon_px.astype(np.float32)))
                areas.append(area)
                if area < 20:
                    warnings.append(f"tiny mask ({area:.0f}px): {split}/{stem}")
                contrast = visible_contrast(image, polygon_px)
                contrasts.append(contrast)
                image_min_contrast = min(image_min_contrast, contrast)
                if args.min_visible_contrast > 0 and contrast < args.min_visible_contrast:
                    split_low_images.add(stem)
                    low_contrast_records.append({"split": split, "image": image_path.name,
                                                 "instance": instance_index, "contrast": contrast,
                                                 "area": area})

            digest = hashlib.sha256(image_path.read_bytes()).hexdigest()
            if digest in seen_sha:
                errors.append(f"exact duplicate: {image_path} == {seen_sha[digest]}")
            else:
                seen_sha[digest] = str(image_path)
            perceptual = average_hash(image)
            if any(np.count_nonzero(perceptual != old) <= 1 for old in recent_hashes[-5:]):
                warnings.append(f"near-duplicate consecutive frame: {split}/{stem}")
            recent_hashes.append(perceptual)

            parts = stem.split("_")
            if len(parts) >= 3:
                sessions["_".join(parts[1:-2])].add(split)
            item = metadata.get(image_path.name)
            if item is None:
                if args.require_metadata:
                    errors.append(f"missing metadata: {split}/{image_path.name}")
            else:
                if int(item.get("instances", len(instances))) != len(instances):
                    errors.append(f"metadata/label instance mismatch: {split}/{image_path.name}")
                scene_splits[scene_group(item)].add(split)
            preview_candidates.append({"split": split, "image_path": str(image_path),
                                       "instances": instances, "stem": stem,
                                       "min_contrast": image_min_contrast})

        stats["splits"][split] = {
            "images": len(images), "positive_images": positives, "negative_images": negatives,
            "instances": instances_count,
            "mask_area_px": {"min": min(areas) if areas else 0,
                             "median": float(np.median(areas)) if areas else 0,
                             "max": max(areas) if areas else 0},
            "visible_contrast": percentile_summary(contrasts),
            "low_contrast_instances": sum(record["split"] == split for record in low_contrast_records),
            "low_contrast_images": len(split_low_images),
        }

    session_leakage = {session: sorted(used) for session, used in sessions.items() if len(used) > 1}
    # With metadata, randomizer seed+cycle is the true independent scene unit;
    # one long collection session may legitimately contribute different cycles
    # to different splits. Fall back to filename sessions only without metadata.
    if not scene_splits:
        for session, used in session_leakage.items():
            errors.append(f"session leakage: {session} appears in {used}")
    leaking_scenes = {group: sorted(used) for group, used in scene_splits.items() if len(used) > 1}
    if leaking_scenes:
        examples = list(leaking_scenes.items())[:8]
        message = f"scene leakage: {len(leaking_scenes)} scene group(s), examples={examples}"
        (errors if args.fail_on_scene_leakage else warnings).append(message)
    if args.fail_on_low_contrast and low_contrast_records:
        errors.append(f"low-contrast labels: {len(low_contrast_records)} instance(s) in "
                      f"{len({(r['split'], r['image']) for r in low_contrast_records})} image(s)")
    for record in sorted(low_contrast_records, key=lambda item: item["contrast"])[:50]:
        warnings.append("low-contrast mask ({:.2f}): {}/{} instance={}".format(
            record["contrast"], record["split"], record["image"], record["instance"]))

    stats["total_images"] = sum(value.get("images", 0) for value in stats["splits"].values())
    if args.require_training_ready:
        for split in ("train", "val", "test"):
            split_stats = stats["splits"].get(split, {})
            if split_stats.get("images", 0) == 0:
                errors.append(f"empty split: {split}")
            elif split_stats.get("positive_images", 0) == 0 or split_stats.get("negative_images", 0) == 0:
                errors.append(f"split must contain positive and negative images: {split}")
    stats["session_count"] = len(sessions)
    stats["session_leakage"] = session_leakage
    stats["scene_group_count"] = len(scene_splits)
    stats["scene_leakage"] = leaking_scenes
    stats["semantic_visibility"] = {
        "threshold": args.min_visible_contrast,
        "low_contrast_instances": len(low_contrast_records),
        "low_contrast_images": len({(r["split"], r["image"]) for r in low_contrast_records}),
        "lowest_examples": sorted(low_contrast_records, key=lambda item: item["contrast"])[:100],
    }
    stats["warning_counts"] = dict(Counter(warning.split(":", 1)[0] for warning in warnings))

    preview_dir = root / "previews"
    preview_dir.mkdir(parents=True, exist_ok=True)
    for old_preview in preview_dir.glob("*.jpg"):
        old_preview.unlink()
    count = min(max(args.previews, 0), len(preview_candidates))
    suspicious = sorted(preview_candidates, key=lambda item: item["min_contrast"])
    chosen = suspicious[:min(count // 2, len(low_contrast_records))]
    chosen_stems = {(item["split"], item["stem"]) for item in chosen}
    remaining = [item for item in preview_candidates if (item["split"], item["stem"]) not in chosen_stems]
    rng = random.Random(args.seed)
    chosen.extend(rng.sample(remaining, min(count-len(chosen), len(remaining))))
    for item in chosen:
        image = cv2.imread(item["image_path"])
        if image is None:
            continue
        height, width = image.shape[:2]
        overlay = image.copy()
        for polygon in item["instances"]:
            polygon_px = np.round(polygon * [width, height]).astype(np.int32)
            cv2.fillPoly(overlay, [polygon_px], (0, 255, 0))
            cv2.polylines(image, [polygon_px], True, (0, 255, 255), 2)
        image = cv2.addWeighted(image, 0.7, overlay, 0.3, 0)
        cv2.imwrite(str(preview_dir / f"{item['split']}_{item['stem']}.jpg"), image)

    (root / "audit_report.json").write_text(json.dumps(stats, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(stats, indent=2, ensure_ascii=False))
    if errors:
        raise SystemExit(f"dataset audit failed with {len(errors)} error(s)")


if __name__ == "__main__":
    main()
