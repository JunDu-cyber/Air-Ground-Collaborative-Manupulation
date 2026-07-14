#!/usr/bin/env python3
"""Build a clean, scene-isolated mine dataset without modifying the source."""

import argparse
import hashlib
import json
import os
import random
import shutil
import subprocess
import sys
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np
import yaml


SPLITS = ("train", "val", "test")


def parse_label(path):
    instances = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        if not raw.strip():
            continue
        fields = raw.split()
        if len(fields) < 7 or (len(fields)-1) % 2 or int(fields[0]) != 0:
            raise ValueError(f"invalid segmentation label: {path}")
        polygon = np.asarray(list(map(float, fields[1:])), np.float32).reshape(-1, 2)
        if not np.isfinite(polygon).all() or (polygon < 0).any() or (polygon > 1).any():
            raise ValueError(f"polygon outside [0,1]: {path}")
        instances.append(polygon)
    return instances


def visible_contrast(image, polygon_px):
    height, width = image.shape[:2]
    mask = np.zeros((height, width), np.uint8)
    cv2.fillPoly(mask, [np.round(polygon_px).astype(np.int32)], 1)
    outer = cv2.dilate(mask, np.ones((9, 9), np.uint8), iterations=1) - mask
    inside = image[mask.astype(bool)].astype(np.float32)
    outside = image[outer.astype(bool)].astype(np.float32)
    if not len(inside) or not len(outside):
        return float("inf")
    return float(np.mean(np.abs(inside - np.median(outside, axis=0))))


def scene_group(item):
    domain = item.get("source_domain") or item.get("domain_tag") or "unknown"
    cycle = int(item.get("domain_cycle", -1))
    seed = int(item.get("randomizer_seed", -1))
    session = str(item.get("session", "unknown"))
    if seed >= 0:
        run = f"seed_{seed}"
    elif session.startswith("curated_"):
        run = f"legacy_{domain}"
    else:
        run = session
    return f"{domain}|{run}|cycle_{cycle}"


def load_metadata(root):
    metadata = {}
    for split in SPLITS:
        path = root / "metadata" / f"{split}.jsonl"
        if not path.is_file():
            continue
        for raw in path.read_text(encoding="utf-8").splitlines():
            if raw.strip():
                item = json.loads(raw)
                metadata[item["image"]] = item
    return metadata


def assignment_objective(assignment, groups, ratios):
    totals = {"images": sum(group["images"] for group in groups.values()),
              "positives": sum(group["positives"] for group in groups.values()),
              "instances": sum(group["instances"] for group in groups.values())}
    domains = sorted({group["domain"] for group in groups.values()})
    domain_totals = {domain: sum(group["images"] for group in groups.values() if group["domain"] == domain)
                     for domain in domains}
    counts = {split: {"images": 0, "positives": 0, "instances": 0,
                      "domains": Counter(), "groups": 0} for split in SPLITS}
    for key, split in assignment.items():
        group = groups[key]
        counts[split]["groups"] += 1
        for field in ("images", "positives", "instances"):
            counts[split][field] += group[field]
        counts[split]["domains"][group["domain"]] += group["images"]
    score = 0.0
    weights = {"images": 5.0, "positives": 1.5, "instances": 1.0}
    for index, split in enumerate(SPLITS):
        ratio = ratios[index]
        if counts[split]["groups"] == 0 or counts[split]["images"] == 0:
            score += 1e6
            continue
        if counts[split]["positives"] == 0 or counts[split]["positives"] == counts[split]["images"]:
            score += 1e5
        for field, weight in weights.items():
            target = max(totals[field] * ratio, 1.0)
            score += weight * ((counts[split][field]-target)/target) ** 2
        for domain in domains:
            target = max(domain_totals[domain] * ratio, 1.0)
            score += 0.35 * ((counts[split]["domains"][domain]-target)/target) ** 2
    return score


def assign_groups(groups, ratios, seed):
    if len(groups) < 3:
        raise SystemExit("at least three independent scene groups are required")
    keys = sorted(groups)
    rng = random.Random(seed)
    best_assignment, best_score = None, float("inf")
    # Multi-start group assignment is inexpensive (normally tens of scenes)
    # and handles large, unequal cycles better than fixed 500-frame blocks.
    for _ in range(max(4000, len(keys)*300)):
        assignment = {key: rng.choices(SPLITS, weights=ratios, k=1)[0] for key in keys}
        score = assignment_objective(assignment, groups, ratios)
        if score < best_score:
            best_assignment, best_score = assignment, score
    return best_assignment, best_score


def link_or_copy(source, target, copy_files):
    if copy_files:
        shutil.copy2(source, target)
        return "copy"
    try:
        os.link(source, target)
        return "hardlink"
    except OSError:
        shutil.copy2(source, target)
        return "copy-fallback"


def main():
    parser = argparse.ArgumentParser(
        description="Create a non-destructive, clean and scene-isolated YOLO dataset")
    parser.add_argument("dataset_dir")
    parser.add_argument("--output-dir", default=None,
                        help="default: sibling directory named <dataset>_prepared")
    parser.add_argument("--min-visible-contrast", type=float, default=3.0)
    parser.add_argument("--min-mask-area", type=float, default=80.0)
    parser.add_argument("--drop-border-masks", action="store_true")
    parser.add_argument("--copy", action="store_true",
                        help="copy files instead of space-saving hard links")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--train-ratio", type=float, default=0.70)
    parser.add_argument("--val-ratio", type=float, default=0.15)
    parser.add_argument("--test-ratio", type=float, default=0.15)
    parser.add_argument("--skip-output-audit", action="store_true")
    args = parser.parse_args()

    root = Path(args.dataset_dir).expanduser().resolve()
    output = (Path(args.output_dir).expanduser().resolve() if args.output_dir else
              root.with_name(root.name + "_prepared"))
    if output == root:
        raise SystemExit("output directory must differ from source; in-place curation is intentionally disabled")
    if output.exists() and any(output.iterdir()):
        raise SystemExit(f"output directory is not empty: {output}")
    ratios = (args.train_ratio, args.val_ratio, args.test_ratio)
    if any(value <= 0 for value in ratios) or not np.isclose(sum(ratios), 1.0):
        raise SystemExit("train/val/test ratios must be positive and sum to 1")
    metadata = load_metadata(root)
    if not metadata:
        raise SystemExit("metadata/*.jsonl is required for leakage-safe scene grouping")

    records = []
    for source_split in SPLITS:
        for image_path in sorted((root / "images" / source_split).glob("*.jpg")):
            label_path = root / "labels" / source_split / f"{image_path.stem}.txt"
            if not label_path.is_file():
                raise SystemExit(f"missing label: {label_path}")
            item = metadata.get(image_path.name)
            if item is None:
                raise SystemExit(f"missing metadata: {image_path.name}")
            image = cv2.imread(str(image_path))
            if image is None:
                raise SystemExit(f"unreadable image: {image_path}")
            height, width = image.shape[:2]
            instances = parse_label(label_path)
            reasons = []
            metrics = []
            for polygon in instances:
                polygon_px = polygon * [width, height]
                area = float(abs(cv2.contourArea(polygon_px.astype(np.float32))))
                contrast = visible_contrast(image, polygon_px)
                border = (np.min(polygon_px[:, 0]) <= 1.5 or np.max(polygon_px[:, 0]) >= width-1.5 or
                          np.min(polygon_px[:, 1]) <= 1.5 or np.max(polygon_px[:, 1]) >= height-1.5)
                metrics.append({"area": area, "contrast": contrast, "border": bool(border)})
                if args.min_visible_contrast > 0 and contrast < args.min_visible_contrast:
                    reasons.append("low_visible_contrast")
                if args.min_mask_area > 0 and area < args.min_mask_area:
                    reasons.append("tiny_mask")
                if args.drop_border_masks and border:
                    reasons.append("border_mask")
            records.append({
                "image": image_path, "label": label_path, "metadata": item,
                "instances": len(instances), "positive": int(bool(instances)),
                "label_text": label_path.read_text(encoding="utf-8"),
                "sha256": hashlib.sha256(image_path.read_bytes()).hexdigest(),
                "scene_group": scene_group(item),
                "domain": item.get("source_domain") or item.get("domain_tag") or "unknown",
                "reasons": reasons, "metrics": metrics,
            })

    by_hash = defaultdict(list)
    for record in records:
        by_hash[record["sha256"]].append(record)
    for duplicate_group in by_hash.values():
        if len(duplicate_group) < 2:
            continue
        label_versions = {record["label_text"] for record in duplicate_group}
        if len(label_versions) > 1:
            for record in duplicate_group:
                record["reasons"].append("inconsistent_exact_duplicate")
        else:
            for record in sorted(duplicate_group, key=lambda item: item["image"].name)[1:]:
                record["reasons"].append("exact_duplicate")

    kept = [record for record in records if not record["reasons"]]
    dropped = [record for record in records if record["reasons"]]
    groups = {}
    grouped_records = defaultdict(list)
    for record in kept:
        grouped_records[record["scene_group"]].append(record)
    for key, items in grouped_records.items():
        groups[key] = {"images": len(items), "positives": sum(item["positive"] for item in items),
                       "instances": sum(item["instances"] for item in items),
                       "domain": items[0]["domain"]}
    assignment, objective = assign_groups(groups, ratios, args.seed)

    for split in SPLITS:
        (output / "images" / split).mkdir(parents=True, exist_ok=True)
        (output / "labels" / split).mkdir(parents=True, exist_ok=True)
    (output / "metadata").mkdir(parents=True, exist_ok=True)
    output_metadata = {split: [] for split in SPLITS}
    link_modes = Counter()
    split_counts = {split: {"images": 0, "positives": 0, "negatives": 0, "instances": 0}
                    for split in SPLITS}
    for record in kept:
        split = assignment[record["scene_group"]]
        link_modes[link_or_copy(record["image"], output/"images"/split/record["image"].name, args.copy)] += 1
        link_modes[link_or_copy(record["label"], output/"labels"/split/record["label"].name, args.copy)] += 1
        item = dict(record["metadata"])
        item.update({"image": record["image"].name, "split": split,
                     "scene_group": record["scene_group"],
                     "source_domain": record["domain"]})
        output_metadata[split].append(item)
        split_counts[split]["images"] += 1
        split_counts[split]["positives"] += record["positive"]
        split_counts[split]["negatives"] += 1-record["positive"]
        split_counts[split]["instances"] += record["instances"]

    for split, items in output_metadata.items():
        with (output/"metadata"/f"{split}.jsonl").open("w", encoding="utf-8") as handle:
            for item in sorted(items, key=lambda value: value["image"]):
                handle.write(json.dumps(item, ensure_ascii=False) + "\n")
    with (output/"mine_dataset.yaml").open("w", encoding="utf-8") as handle:
        yaml.safe_dump({"path": str(output), "train": "images/train", "val": "images/val",
                        "test": "images/test", "names": {0: "landmine"}}, handle, sort_keys=False)

    drop_counts = Counter(reason for record in dropped for reason in set(record["reasons"]))
    report = {
        "source": str(root), "output": str(output), "source_images": len(records),
        "kept_images": len(kept), "dropped_images": len(dropped),
        "drop_reason_counts": dict(sorted(drop_counts.items())),
        "filter": {"min_visible_contrast": args.min_visible_contrast,
                   "min_mask_area": args.min_mask_area, "drop_border_masks": args.drop_border_masks},
        "split_ratios": dict(zip(SPLITS, ratios)), "split_counts": split_counts,
        "scene_groups": {key: {**groups[key], "split": assignment[key]} for key in sorted(groups)},
        "assignment_objective": objective, "link_modes": dict(link_modes),
        "dropped_examples": [{"image": record["image"].name,
                              "reasons": sorted(set(record["reasons"])),
                              "metrics": record["metrics"]} for record in dropped[:200]],
    }
    (output/"curation_report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    if not args.skip_output_audit:
        audit_script = Path(__file__).with_name("audit_mine_dataset.py")
        subprocess.run([sys.executable, str(audit_script), str(output), "--previews", "24",
                        "--min-visible-contrast", str(args.min_visible_contrast),
                        "--fail-on-low-contrast", "--fail-on-scene-leakage",
                        "--require-metadata", "--require-training-ready"], check=True)
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
