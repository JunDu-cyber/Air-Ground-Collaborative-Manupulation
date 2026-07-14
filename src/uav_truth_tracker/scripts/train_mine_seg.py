#!/usr/bin/env python3
"""Train, evaluate, and export the one-class landmine YOLO segmentation model."""

import argparse
import csv
import hashlib
import json
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


def best_f1_operating_point(metric):
    """Extract the validation confidence that maximizes mean mask F1."""
    curves = metric.curves_results
    values = {}
    confidence_axis = None
    for x, y, x_label, y_label in curves:
        if x_label == "Confidence":
            confidence_axis = np.asarray(x, dtype=np.float64)
            curve = np.asarray(y, dtype=np.float64)
            values[y_label] = np.mean(curve, axis=0) if curve.ndim > 1 else curve
    if confidence_axis is None or "F1" not in values or not values["F1"].size:
        return None
    index = int(np.nanargmax(values["F1"]))
    precision = values.get("Precision")
    recall = values.get("Recall")
    return {"confidence": float(confidence_axis[index]), "f1": float(values["F1"][index]),
            "precision": float(precision[index]) if precision is not None and index < precision.size else None,
            "recall": float(recall[index]) if recall is not None and index < recall.size else None}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dataset_dir")
    ap.add_argument("--model", default="yolo11s-seg.pt")
    ap.add_argument("--epochs", type=int, default=200)
    ap.add_argument("--imgsz", type=int, default=960)
    ap.add_argument("--batch", type=int, default=-1,
                    help="Ultralytics batch; -1 selects an automatic GPU batch")
    ap.add_argument("--device", default="0")
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--project", default="~/mine_yolo_runs")
    ap.add_argument("--name", default="mine_seg")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--patience", type=int, default=50)
    ap.add_argument("--optimizer", default="AdamW")
    ap.add_argument("--lr0", type=float, default=0.0015)
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--smoke-test", action="store_true")
    ap.add_argument("--no-export", action="store_true")
    ap.add_argument("--skip-data-audit", action="store_true",
                    help="unsafe escape hatch; normally training requires a passing semantic audit")
    args = ap.parse_args()

    root = Path(args.dataset_dir).expanduser().resolve()
    data = root/"mine_dataset.yaml"
    for relative in ("images/train", "images/val", "images/test",
                     "labels/train", "labels/val", "labels/test"):
        if not (root/relative).is_dir(): raise SystemExit(f"missing dataset directory: {root/relative}")
    if not data.is_file(): raise SystemExit(f"missing dataset yaml: {data}")

    if not args.skip_data_audit:
        audit = Path(__file__).with_name("audit_mine_dataset.py")
        command = [sys.executable, str(audit), str(root), "--previews", "24",
                   "--min-visible-contrast", "3.0", "--fail-on-low-contrast",
                   "--fail-on-scene-leakage", "--require-metadata"]
        command.append("--require-training-ready")
        try:
            subprocess.run(command, check=True, stdout=subprocess.DEVNULL)
        except subprocess.CalledProcessError as exc:
            raise SystemExit(
                f"dataset audit failed; do not train this dataset. See {root/'audit_report.json'}"
            ) from exc

    try:
        import torch
        from ultralytics import YOLO
    except Exception as exc:
        raise SystemExit(f"torch and ultralytics are required: {exc}")
    if args.device != "cpu" and not torch.cuda.is_available():
        raise SystemExit("CUDA device requested but torch.cuda.is_available() is false; use --device cpu for smoke tests")

    epochs, imgsz, workers, fraction = args.epochs, args.imgsz, args.workers, 1.0
    batch = args.batch
    if args.smoke_test:
        epochs, imgsz, workers, fraction, batch = 2, 320, 0, 0.05, 2
        args.name += "_smoke"

    model = YOLO(args.model)
    results = model.train(
        data=str(data), epochs=epochs, imgsz=imgsz, batch=batch, device=args.device,
        project=str(Path(args.project).expanduser()), name=args.name, workers=workers,
        patience=args.patience, seed=args.seed, deterministic=True, fraction=fraction,
        resume=args.resume,
        optimizer=args.optimizer, lr0=args.lr0, lrf=0.01, cos_lr=True,
        hsv_h=0.025, hsv_s=0.65, hsv_v=0.45,
        degrees=180.0, translate=0.10, scale=0.25, shear=2.0, perspective=0.0003,
        fliplr=0.5, flipud=0.5, mosaic=0.35, mixup=0.0, copy_paste=0.10,
        erasing=0.05, close_mosaic=20,
        plots=True, save=True,
    )
    save_dir = Path(model.trainer.save_dir)
    best = save_dir/"weights"/"best.pt"
    trained = YOLO(str(best if best.is_file() else save_dir/"weights"/"last.pt"))
    val_metrics = trained.val(data=str(data), split="val", imgsz=imgsz, device=args.device,
                              project=str(save_dir), name="val_calibration", plots=True)
    operating_point = best_f1_operating_point(val_metrics.seg)
    metrics = trained.val(data=str(data), split="test", imgsz=imgsz, device=args.device,
                          project=str(save_dir), name="test_metrics", plots=True)
    exported = None
    if not args.no_export:
        exported = trained.export(format="onnx", imgsz=imgsz, dynamic=False,
                                  simplify=True, opset=12)
    results_csv = save_dir / "results.csv"
    actual_epochs = 0
    if results_csv.is_file():
        with results_csv.open(encoding="utf-8") as handle:
            actual_epochs = sum(1 for _ in csv.DictReader(handle))
    audit_report = root / "audit_report.json"
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(), "dataset": str(root),
        "model": args.model, "best_checkpoint": str(best), "exported": str(exported) if exported else None,
        "requested_epochs": epochs, "actual_epochs": actual_epochs, "imgsz": imgsz,
        "seed": args.seed, "smoke_test": args.smoke_test,
        "optimizer": args.optimizer, "lr0": args.lr0,
        "recommended_confidence_from_val": operating_point,
        "dataset_audit_sha256": (hashlib.sha256(audit_report.read_bytes()).hexdigest()
                                  if audit_report.is_file() else None),
        "environment": {"python": platform.python_version(), "torch": torch.__version__,
                        "ultralytics": __import__("ultralytics").__version__,
                        "cuda_device": (torch.cuda.get_device_name(0) if torch.cuda.is_available() else "cpu")},
        "metrics": {"box_map50": float(metrics.box.map50), "box_map": float(metrics.box.map),
                    "mask_map50": float(metrics.seg.map50), "mask_map": float(metrics.seg.map),
                    "mask_precision": float(metrics.seg.mp), "mask_recall": float(metrics.seg.mr)},
    }
    (save_dir/"training_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
