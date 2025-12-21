#!/usr/bin/env python3
"""
Generate synthetic training data with domain randomization.

This script captures 1000 images with randomized lighting, materials, and camera parameters
to create a diverse training dataset for object detection and semantic segmentation.

Output: RGB images, depth maps, segmentation masks, bounding boxes (JSON format)

Usage:
    # Inside Isaac Sim Docker container (with humanoid_warehouse.usd scene open)
    ./python.sh /path/to/generate_synthetic_data.py --num-images 1000 --output-dir /root/Documents/synthetic_data

Requirements:
    - Isaac Sim 5.1.0 with humanoid_warehouse.usd scene loaded
    - NVIDIA RTX 3060 (12GB VRAM) recommended
    - Estimated time: 8-12 minutes for 1000 images

Author: Generated for Physical AI & Humanoid Robotics Textbook
Date: 2025-12-19
"""

import argparse
import json
import os
import random
import sys
from pathlib import Path

import cv2
import numpy as np

# Isaac Sim imports
from isaacsim import SimulationApp

# Parse args before launching SimulationApp
parser = argparse.ArgumentParser(description="Generate synthetic training data with domain randomization")
parser.add_argument("--num-images", type=int, default=1000, help="Number of images to generate")
parser.add_argument("--output-dir", type=str, default="/root/Documents/synthetic_data", help="Output directory")
parser.add_argument("--resolution", type=str, default="1280x720", help="Image resolution (WIDTHxHEIGHT)")
args = parser.parse_args()

# Launch Isaac Sim (headless for faster generation)
simulation_app = SimulationApp({"headless": True})  # Set False to visualize

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.synthetic_utils import SyntheticDataHelper
from pxr import Gf, UsdGeom


def randomize_lighting(light_path: str = "/World/Lighting/DirectionalLight"):
    """
    Randomize directional light intensity, color temperature, and angle.

    Args:
        light_path: USD path to directional light prim
    """
    stage = get_current_stage()
    light_prim = stage.GetPrimAtPath(light_path)

    if not light_prim.IsValid():
        print(f"Warning: Light not found at {light_path}, skipping randomization")
        return

    # Randomize intensity (500-2000 lux for indoor/outdoor variation)
    intensity = random.uniform(500, 2000)
    light_prim.GetAttribute("inputs:intensity").Set(intensity)

    # Randomize color temperature (3000K tungsten to 7000K daylight)
    color_temp = random.uniform(3000, 7000)
    light_prim.GetAttribute("inputs:color_temperature").Set(color_temp)

    # Randomize shadow angle (-30° to +30° from vertical)
    angle_x = random.uniform(-30, 30)
    angle_y = random.uniform(-30, 30)
    light_prim.GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3f(angle_x, angle_y, 0))


def randomize_camera_exposure(camera_path: str = "/World/Humanoid/Head/Camera"):
    """
    Randomize camera exposure (±2 EV stops).

    Args:
        camera_path: USD path to camera prim
    """
    stage = get_current_stage()
    camera_prim = stage.GetPrimAtPath(camera_path)

    if not camera_prim.IsValid():
        print(f"Warning: Camera not found at {camera_path}, skipping randomization")
        return

    # Randomize exposure (±2 stops)
    exposure = random.uniform(-2, 2)
    camera_prim.GetAttribute("exposure").Set(exposure)


def randomize_materials(object_paths: list[str]):
    """
    Randomize PBR material properties for specified objects.

    Args:
        object_paths: List of USD paths to objects with materials
    """
    for obj_path in object_paths:
        # Randomize base color
        base_color = (random.random(), random.random(), random.random())

        # Randomize roughness (0.2-0.9)
        roughness = random.uniform(0.2, 0.9)

        # Randomize metallic (0.0-0.5 for most warehouse objects)
        metallic = random.uniform(0.0, 0.5)

        material_path = f"{obj_path}/Material/Shader"

        try:
            omni.kit.commands.execute("ChangeProperty",
                prop_path=f"{material_path}.inputs:diffuse_color_constant",
                value=base_color)
            omni.kit.commands.execute("ChangeProperty",
                prop_path=f"{material_path}.inputs:roughness_constant",
                value=roughness)
            omni.kit.commands.execute("ChangeProperty",
                prop_path=f"{material_path}.inputs:metallic_constant",
                value=metallic)
        except Exception as e:
            # Material path may not exist for all objects
            pass


def generate_synthetic_data(num_images: int, output_dir: str, resolution: tuple[int, int]):
    """
    Generate synthetic training data with domain randomization.

    Args:
        num_images: Number of images to generate
        output_dir: Output directory for images and annotations
        resolution: Image resolution (width, height)
    """
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    os.makedirs(f"{output_dir}/rgb", exist_ok=True)
    os.makedirs(f"{output_dir}/depth", exist_ok=True)
    os.makedirs(f"{output_dir}/segmentation", exist_ok=True)
    os.makedirs(f"{output_dir}/annotations", exist_ok=True)

    print(f"📁 Output directory: {output_dir}")
    print(f"📷 Resolution: {resolution[0]}x{resolution[1]}")
    print(f"🔢 Generating {num_images} images...\n")

    # Initialize synthetic data helper
    sd_helper = SyntheticDataHelper()
    sd_helper.initialize(sensor_names=["Camera"], viewport_name="Viewport")

    # Camera path (adjust based on your scene)
    camera_path = "/World/Humanoid/Head/Camera"

    # Objects to randomize materials (adjust paths based on your scene)
    warehouse_objects = [
        "/World/Warehouse/Floor",
        "/World/Warehouse/Walls",
        "/World/Warehouse/Shelves"
    ]

    # Generate images with domain randomization
    for i in range(num_images):
        # Apply domain randomization
        randomize_lighting()
        randomize_camera_exposure(camera_path)
        randomize_materials(warehouse_objects)

        # Step simulation (1 frame to apply changes)
        omni.kit.app.get_app().update()

        # Capture ground-truth data
        try:
            gt = sd_helper.get_groundtruth(
                [camera_path],
                ["rgb", "depth", "semantic_segmentation", "instance_segmentation", "bounding_box_2d_tight"]
            )

            # Extract data
            rgb_img = gt["rgb"][0]  # (H, W, 3) uint8
            depth_img = gt["depth"][0]  # (H, W) float32 (meters)
            seg_img = gt["semantic_segmentation"][0]  # (H, W) uint8 (class IDs)
            bboxes = gt["bounding_box_2d_tight"][0]  # List of bounding boxes

            # Save RGB image
            rgb_path = f"{output_dir}/rgb/{i:06d}.png"
            cv2.imwrite(rgb_path, cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR))

            # Save depth map (convert to 16-bit PNG, millimeters)
            depth_mm = (depth_img * 1000).astype(np.uint16)  # Convert meters to mm
            depth_path = f"{output_dir}/depth/{i:06d}.png"
            cv2.imwrite(depth_path, depth_mm)

            # Save segmentation mask
            seg_path = f"{output_dir}/segmentation/{i:06d}.png"
            cv2.imwrite(seg_path, seg_img)

            # Save bounding boxes as JSON
            bbox_path = f"{output_dir}/annotations/{i:06d}.json"
            with open(bbox_path, "w") as f:
                json.dump(bboxes, f, indent=2)

            # Progress update every 100 images
            if (i + 1) % 100 == 0:
                print(f"✅ Generated {i + 1}/{num_images} images ({(i + 1) / num_images * 100:.1f}%)")

        except Exception as e:
            print(f"❌ Error generating image {i}: {e}")
            continue

    print(f"\n🎉 Synthetic data generation complete!")
    print(f"📊 Total images: {num_images}")
    print(f"📁 Output: {output_dir}")


def main():
    """Main entry point for synthetic data generation."""
    print("=" * 60)
    print("Isaac Sim Synthetic Data Generation")
    print("=" * 60)
    print(f"Configuration:")
    print(f"  Images: {args.num_images}")
    print(f"  Resolution: {args.resolution}")
    print(f"  Output: {args.output_dir}")
    print("=" * 60 + "\n")

    # Parse resolution
    width, height = map(int, args.resolution.split("x"))

    try:
        # Generate synthetic data
        generate_synthetic_data(args.num_images, args.output_dir, (width, height))

        print("\n" + "=" * 60)
        print("✅ Success!")
        print("=" * 60)
        print("\nNext Steps:")
        print("1. Run export_coco.py to convert to COCO format")
        print("2. Train YOLOv8 on generated dataset")
        print("3. Evaluate on real-world test images")

    except Exception as e:
        print(f"\n❌ Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
    simulation_app.close()
