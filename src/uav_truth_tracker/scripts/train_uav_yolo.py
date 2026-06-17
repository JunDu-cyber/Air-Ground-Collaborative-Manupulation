#!/usr/bin/env python3
"""Train a one-class UAV YOLO detector from a collected Gazebo dataset."""

import argparse
import os

import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "dataset_dir",
        help="Dataset root containing images/train, images/val, labels/train, labels/val",
    )
    parser.add_argument("--model", default="yolo11n.pt")
    parser.add_argument("--epochs", type=int, default=150)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--project", default="~/uav_yolo_runs")
    parser.add_argument("--name", default="uav_detector")
    parser.add_argument("--patience", type=int, default=30,
                        help="Early stopping patience (0 to disable)")
    parser.add_argument("--resume", action="store_true",
                        help="Resume from last checkpoint")
    args = parser.parse_args()

    dataset_dir = os.path.abspath(os.path.expanduser(args.dataset_dir))
    for relative in (
        "images/train",
        "images/val",
        "labels/train",
        "labels/val",
    ):
        path = os.path.join(dataset_dir, relative)
        if not os.path.isdir(path):
            raise SystemExit("missing dataset directory: {}".format(path))

    # Count dataset
    train_count = len(os.listdir(os.path.join(dataset_dir, "images/train")))
    val_count = len(os.listdir(os.path.join(dataset_dir, "images/val")))
    print(f"Dataset: {train_count} train / {val_count} val images")

    dataset_yaml = os.path.join(dataset_dir, "uav_dataset.yaml")
    with open(dataset_yaml, "w", encoding="utf-8") as stream:
        yaml.safe_dump(
            {
                "path": dataset_dir,
                "train": "images/train",
                "val": "images/val",
                "names": {0: "uav"},
            },
            stream,
            sort_keys=False,
        )

    try:
        from ultralytics import YOLO
    except Exception as exc:
        raise SystemExit(
            "ultralytics is required: python3 -m pip install --user ultralytics ({})".format(
                exc
            )
        )

    model = YOLO(args.model)
    model.train(
        data=dataset_yaml,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=os.path.expanduser(args.project),
        name=args.name,
        workers=4,
        patience=args.patience,
        # --- Data augmentation for better generalization ---
        hsv_h=0.02,        # Hue variation (Gazebo lighting changes)
        hsv_s=0.7,         # Saturation variation
        hsv_v=0.4,         # Brightness variation (shadow/sun)
        degrees=15.0,       # Rotation (UAV viewed from different angles)
        translate=0.15,     # Translation
        scale=0.5,          # Scale variation (near/far targets)
        fliplr=0.5,         # Horizontal flip
        flipud=0.0,         # No vertical flip (UAVs don't fly upside down)
        mosaic=1.0,         # Mosaic augmentation
        mixup=0.1,          # MixUp augmentation
        copy_paste=0.1,     # Copy-paste augmentation
        erasing=0.2,        # Random erasing (simulate occlusion)
        crop_fraction=0.8,  # Crop fraction for classification
        resume=args.resume,
    )


if __name__ == "__main__":
    main()
