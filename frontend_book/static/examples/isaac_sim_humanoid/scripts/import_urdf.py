#!/usr/bin/env python3
"""
Import simple_humanoid.urdf into Isaac Sim scene.

This script demonstrates how to import a URDF file into an Isaac Sim scene using the
Omniverse Python API. The imported robot will have all joints, links, and collision
geometries properly configured for physics simulation.

Usage:
    # Inside Isaac Sim Docker container
    ./python.sh /path/to/import_urdf.py

Requirements:
    - Isaac Sim 5.1.0 or later
    - simple_humanoid.urdf in ../urdf/ directory
    - NVIDIA RTX GPU with 6GB+ VRAM

Author: Generated for Physical AI & Humanoid Robotics Textbook
Date: 2025-12-19
"""

import os
import sys
from pathlib import Path

# Isaac Sim imports
from isaacsim import SimulationApp

# Launch Isaac Sim with GUI (set headless=True for no GUI)
simulation_app = SimulationApp({"headless": False})

import omni
from omni.isaac.core.utils.extensions import enable_extension

# Enable URDF importer extension
enable_extension("omni.importer.urdf")

from omni.importer.urdf import _urdf as urdf_importer
from pxr import Gf, UsdGeom, UsdPhysics

def import_humanoid_urdf(urdf_path: str, dest_path: str = "/World/Humanoid") -> str:
    """
    Import URDF file into Isaac Sim scene.

    Args:
        urdf_path: Absolute path to .urdf file
        dest_path: USD scene path where robot will be placed

    Returns:
        USD prim path of imported robot root

    Raises:
        FileNotFoundError: If URDF file doesn't exist
        RuntimeError: If import fails
    """
    # Validate URDF file exists
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")

    print(f"Importing URDF from: {urdf_path}")
    print(f"Destination path: {dest_path}")

    # Configure URDF import settings
    import_config = urdf_importer.ImportConfig()
    import_config.merge_fixed_joints = False  # Keep all joints separate
    import_config.fix_base = False  # Allow robot to move (not fixed to ground)
    import_config.import_inertia_tensor = True  # Import mass/inertia for physics
    import_config.default_drive_type = urdf_importer.UrdfJointTargetType.JOINT_DRIVE_POSITION
    import_config.distance_scale = 1.0  # 1.0 = meters (URDF units)
    import_config.density = 1000.0  # kg/m³ (if URDF doesn't specify mass)

    # Import URDF
    print("Importing URDF (this may take 30-60 seconds)...")
    result, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        dest_path=dest_path
    )

    if not result:
        raise RuntimeError(f"URDF import failed for: {urdf_path}")

    print(f"✅ URDF imported successfully at: {prim_path}")

    # Verify robot prim exists
    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(prim_path)

    if not robot_prim.IsValid():
        raise RuntimeError(f"Imported prim is invalid: {prim_path}")

    # Print joint hierarchy
    print("\n📋 Joint Hierarchy:")
    for prim in robot_prim.GetAllChildren():
        if prim.IsA(UsdPhysics.Joint):
            print(f"  - {prim.GetPath().name}: {prim.GetTypeName()}")

    return prim_path


def configure_robot_physics(robot_path: str):
    """
    Configure physics properties for imported robot.

    Args:
        robot_path: USD path to robot root prim
    """
    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(robot_path)

    # Enable self-collision (prevents robot limbs from intersecting)
    physics_api = UsdPhysics.RigidBodyAPI.Apply(robot_prim)
    physics_api.CreateRigidBodyEnabledAttr(True)

    print(f"✅ Physics configured for: {robot_path}")


def main():
    """Main entry point for URDF import script."""
    # Get URDF path (relative to this script)
    script_dir = Path(__file__).parent
    urdf_path = script_dir.parent / "urdf" / "simple_humanoid.urdf"
    urdf_path = str(urdf_path.resolve())

    print("=" * 60)
    print("Isaac Sim URDF Import Script")
    print("=" * 60)

    try:
        # Import URDF
        robot_path = import_humanoid_urdf(urdf_path)

        # Configure physics
        configure_robot_physics(robot_path)

        print("\n" + "=" * 60)
        print("✅ Import Complete!")
        print("=" * 60)
        print("\nNext Steps:")
        print("1. Click Play (▶) button to start physics simulation")
        print("2. Robot should stand upright (or fall if no controller)")
        print("3. Use Create > Camera to add camera for synthetic data")
        print("\n💡 Tip: Save scene as .usd file for future use")

    except Exception as e:
        print(f"\n❌ Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()

    # Keep simulation running (close window to exit)
    while simulation_app.is_running():
        simulation_app.update()

    simulation_app.close()
