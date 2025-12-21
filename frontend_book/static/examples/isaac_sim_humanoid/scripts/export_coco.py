#!/usr/bin/env python3
"""
Export Isaac Sim synthetic data to COCO format.

This script converts raw Isaac Sim annotations (bounding box JSONs) into COCO format
for training object detection models like YOLOv8, Faster R-CNN, or Mask R-CNN.

COCO Format: http://cocodataset.org/#format-data

Usage:
    python export_coco.py --input-dir /path/to/synthetic_data --output coco_annotations.json

Requirements:
    - Python 3.8+
    - Generated synthetic data from generate_synthetic_data.py

Author: Generated for Physical AI & Humanoid Robotics Textbook
Date: 2025-12-19
"""

import argparse
import glob
import json
import os
from datetime import datetime
from pathlib import Path


def convert_to_coco(input_dir: str, output_file: str, categories: list[dict]):
    """
    Convert Isaac Sim annotations to COCO format.

    Args:
        input_dir: Directory containing synthetic data (rgb/, annotations/ subdirs)
        output_file: Output COCO JSON file path
        categories: List of category dicts [{"id": 1, "name": "person", "supercategory": "human"}]

    Returns:
        COCO-format dictionary
    """
    # Initialize COCO structure
    coco = {
        "info": {
            "description": "Synthetic Humanoid Training Dataset (Isaac Sim)",
            "url": "https://github.com/yourusername/humanoid-robotics",
            "version": "1.0",
            "year": 2025,
            "contributor": "Physical AI & Humanoid Robotics Textbook",
            "date_created": datetime.now().isoformat()
        },
        "licenses": [
            {
                "id": 1,
                "name": "Attribution-NonCommercial-ShareAlike License",
                "url": "http://creativecommons.org/licenses/by-nc-sa/4.0/"
            }
        ],
        "images": [],
        "annotations": [],
        "categories": categories
    }

    # Find all annotation files
    annotation_files = sorted(glob.glob(os.path.join(input_dir, "annotations", "*.json")))

    if len(annotation_files) == 0:
        raise FileNotFoundError(f"No annotation files found in {input_dir}/annotations/")

    print(f"📁 Found {len(annotation_files)} annotation files")

    annotation_id = 1

    for img_id, annotation_file in enumerate(annotation_files, start=1):
        # Load annotation
        with open(annotation_file, "r") as f:
            isaac_annotations = json.load(f)

        # Get corresponding RGB image
        image_basename = Path(annotation_file).stem  # e.g., "000000"
        rgb_file = os.path.join(input_dir, "rgb", f"{image_basename}.png")

        if not os.path.exists(rgb_file):
            print(f"⚠️  Warning: RGB image not found for {annotation_file}, skipping")
            continue

        # Get image dimensions (from first image, assume all are same resolution)
        if img_id == 1:
            import cv2
            img = cv2.imread(rgb_file)
            image_height, image_width = img.shape[:2]
            print(f"📷 Image resolution: {image_width}x{image_height}")

        # Add image entry
        coco["images"].append({
            "id": img_id,
            "file_name": f"rgb/{image_basename}.png",  # Relative path
            "width": image_width,
            "height": image_height,
            "license": 1,
            "date_captured": datetime.now().isoformat()
        })

        # Process bounding boxes from Isaac Sim
        for bbox_data in isaac_annotations:
            # Isaac Sim bbox format: {"coords": [x_min, y_min, x_max, y_max], "label": "humanoid"}
            if "coords" not in bbox_data:
                continue

            x_min, y_min, x_max, y_max = bbox_data["coords"]
            width = x_max - x_min
            height = y_max - y_min
            area = width * height

            # Skip invalid bounding boxes (zero area)
            if area <= 0:
                continue

            # Get category ID from label
            label = bbox_data.get("label", "humanoid")
            category_id = next((cat["id"] for cat in categories if cat["name"] == label), 1)

            # Add annotation entry (COCO format)
            coco["annotations"].append({
                "id": annotation_id,
                "image_id": img_id,
                "category_id": category_id,
                "bbox": [x_min, y_min, width, height],  # COCO: [x, y, width, height]
                "area": area,
                "iscrowd": 0,
                "segmentation": []  # Empty for bounding box only (add polygon for instance segmentation)
            })

            annotation_id += 1

        # Progress update
        if img_id % 100 == 0:
            print(f"✅ Processed {img_id}/{len(annotation_files)} images")

    print(f"\n📊 COCO Dataset Statistics:")
    print(f"  Images: {len(coco['images'])}")
    print(f"  Annotations: {len(coco['annotations'])}")
    print(f"  Categories: {len(coco['categories'])}")

    # Save COCO JSON
    with open(output_file, "w") as f:
        json.dump(coco, f, indent=2)

    print(f"\n💾 COCO annotations saved to: {output_file}")

    return coco


def main():
    """Main entry point for COCO export script."""
    parser = argparse.ArgumentParser(description="Export Isaac Sim annotations to COCO format")
    parser.add_argument("--input-dir", type=str, required=True, help="Input directory with synthetic data")
    parser.add_argument("--output", type=str, default="coco_annotations.json", help="Output COCO JSON file")
    parser.add_argument("--categories", type=str, default="humanoid", help="Comma-separated category names (e.g., 'humanoid,obstacle,floor')")
    args = parser.parse_args()

    print("=" * 60)
    print("Isaac Sim to COCO Format Converter")
    print("=" * 60)
    print(f"Input: {args.input_dir}")
    print(f"Output: {args.output}")
    print("=" * 60 + "\n")

    # Parse categories
    category_names = [name.strip() for name in args.categories.split(",")]
    categories = [
        {"id": i + 1, "name": name, "supercategory": "robot"}
        for i, name in enumerate(category_names)
    ]

    print(f"📂 Categories: {category_names}\n")

    try:
        # Convert to COCO
        coco = convert_to_coco(args.input_dir, args.output, categories)

        print("\n" + "=" * 60)
        print("✅ Export Complete!")
        print("=" * 60)
        print("\nNext Steps:")
        print("1. Validate COCO JSON with pycocotools:")
        print("   from pycocotools.coco import COCO")
        print("   coco = COCO('coco_annotations.json')")
        print("\n2. Train YOLOv8 object detection:")
        print("   yolo train data=coco.yaml model=yolov8n.pt epochs=100")
        print("\n3. Visualize annotations:")
        print("   python visualize_coco.py --coco coco_annotations.json")

    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        exit(1)


if __name__ == "__main__":
    main()
