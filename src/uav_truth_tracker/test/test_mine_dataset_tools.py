#!/usr/bin/env python3

import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np


PACKAGE = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class MineDatasetToolsTest(unittest.TestCase):
    def test_mesh_and_triangle_raster(self):
        collector = load("mine_collector", PACKAGE/"scripts"/"collect_mine_seg_dataset_node.py")
        vertices, triangles = collector.mine_mesh()
        self.assertEqual(vertices.shape, (74, 3))
        self.assertEqual(triangles.shape, (140, 3))
        zbuf = np.full((100, 100), np.inf, np.float32)
        collector.MineSegDatasetCollector._raster_triangle(
            zbuf, np.array([[20.,20.], [80.,20.], [50.,80.]]), np.array([2.,2.,2.]))
        self.assertGreater(np.isfinite(zbuf).sum(), 1000)

    def test_auditor_accepts_valid_segmentation_dataset(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for index, split in enumerate(("train", "val", "test")):
                (root/"images"/split).mkdir(parents=True)
                (root/"labels"/split).mkdir(parents=True)
                name = f"{index:07d}_session{index}_000_1234567890_000000000"
                image = np.full((80, 100, 3), 30 + index*40, np.uint8)
                cv2.circle(image, (50,40), 12, (0,0,220), -1)
                cv2.imwrite(str(root/"images"/split/(name+".jpg")), image)
                (root/"labels"/split/(name+".txt")).write_text(
                    "0 0.38 0.50 0.50 0.35 0.62 0.50 0.50 0.65\n", encoding="utf-8")
            subprocess.run([sys.executable, str(PACKAGE/"scripts"/"audit_mine_dataset.py"),
                            str(root), "--previews", "3"], check=True,
                           stdout=subprocess.DEVNULL)
            self.assertTrue((root/"audit_report.json").is_file())
            self.assertEqual(len(list((root/"previews").glob("*.jpg"))), 3)

    def test_auditor_rejects_label_on_uniform_background(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for split in ("train", "val", "test"):
                (root/"images"/split).mkdir(parents=True)
                (root/"labels"/split).mkdir(parents=True)
            stem = "0000000_bad_0000000000_000000000"
            cv2.imwrite(str(root/"images/train"/(stem+".jpg")), np.full((80, 100, 3), 60, np.uint8))
            (root/"labels/train"/(stem+".txt")).write_text(
                "0 0.38 0.50 0.50 0.35 0.62 0.50 0.50 0.65\n", encoding="utf-8")
            result = subprocess.run(
                [sys.executable, str(PACKAGE/"scripts"/"audit_mine_dataset.py"), str(root),
                 "--previews", "0", "--fail-on-low-contrast"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.assertNotEqual(result.returncode, 0)
            report = json.loads((root/"audit_report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["semantic_visibility"]["low_contrast_instances"], 1)

    def test_curator_is_non_destructive_and_keeps_scenes_isolated(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)/"raw"
            output = Path(tmp)/"prepared"
            metadata = {split: [] for split in ("train", "val", "test")}
            for split in metadata:
                (root/"images"/split).mkdir(parents=True)
                (root/"labels"/split).mkdir(parents=True)
            (root/"metadata").mkdir(parents=True)
            source_count = 0
            for cycle in range(6):
                for view in range(3):
                    split = ("train", "val", "test")[(cycle+view) % 3]
                    stem = f"{source_count:07d}_collection_0000000000_{source_count:09d}"
                    image = np.full((80, 100, 3), 35+cycle*5, np.uint8)
                    center = (35+view*14, 40)
                    is_positive = view < 2
                    if is_positive:
                        cv2.circle(image, center, 10, (10, 20+cycle*8, 210), -1)
                    else:
                        image[5+cycle, 5+cycle] = (80, 80, 80)
                    cv2.imwrite(str(root/"images"/split/(stem+".jpg")), image)
                    x, y = center
                    label = ("0 " + " ".join(str(value) for value in (
                        (x-11)/100, y/80, x/100, (y-11)/80,
                        (x+11)/100, y/80, x/100, (y+11)/80)) + "\n") if is_positive else ""
                    (root/"labels"/split/(stem+".txt")).write_text(label, encoding="utf-8")
                    metadata[split].append({"image": stem+".jpg", "split": split,
                                            "session": "collection", "source_domain": "gray",
                                            "domain_cycle": cycle, "randomizer_seed": 123,
                                            "instances": int(is_positive),
                                            "mines": ([{}] if is_positive else [])})
                    source_count += 1
            for split, items in metadata.items():
                with (root/"metadata"/(split+".jsonl")).open("w", encoding="utf-8") as handle:
                    for item in items:
                        handle.write(json.dumps(item) + "\n")
            subprocess.run(
                [sys.executable, str(PACKAGE/"scripts"/"curate_mine_dataset.py"), str(root),
                 "--output-dir", str(output), "--min-visible-contrast", "1.0",
                 "--min-mask-area", "20"],
                check=True, stdout=subprocess.DEVNULL)
            self.assertEqual(len(list(root.glob("images/*/*.jpg"))), source_count)
            self.assertEqual(len(list(output.glob("images/*/*.jpg"))), source_count)
            scene_splits = {}
            for split in ("train", "val", "test"):
                for raw in (output/"metadata"/(split+".jsonl")).read_text().splitlines():
                    item = json.loads(raw)
                    scene_splits.setdefault(item["scene_group"], set()).add(split)
            self.assertTrue(scene_splits)
            self.assertTrue(all(len(splits) == 1 for splits in scene_splits.values()))
            audit = json.loads((output/"audit_report.json").read_text(encoding="utf-8"))
            self.assertEqual(audit["errors"], [])


if __name__ == "__main__":
    unittest.main()
