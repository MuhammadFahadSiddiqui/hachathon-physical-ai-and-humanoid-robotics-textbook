#!/usr/bin/env python3
"""
Action Primitives for Robot Task Planning

Defines the 12 atomic robot behaviors that LLMs can use to compose task plans.
Each primitive includes JSON schema validation, parameter bounds checking, and
precondition verification.

Author: Module 4 - Vision-Language-Action (VLA)
License: MIT
"""

from typing import Dict, List, Any, Optional
from dataclasses import dataclass
import json


@dataclass
class ActionPrimitive:
    """Base class for all action primitives."""
    action: str
    parameters: Dict[str, Any]
    expected_duration: float

    def validate(self) -> tuple[bool, Optional[str]]:
        """
        Validate action parameters against schema.
        Returns (is_valid, error_message).
        """
        raise NotImplementedError("Subclasses must implement validate()")

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            "action": self.action,
            "parameters": self.parameters,
            "expected_duration": self.expected_duration
        }


class Navigate(ActionPrimitive):
    """Navigate to a target location."""

    KNOWN_LOCATIONS = ["kitchen", "bedroom", "living_room", "bathroom",
                       "table", "shelf", "charging_station", "home", "user"]

    def __init__(self, target: str | List[float], expected_duration: float = 15.0):
        super().__init__(
            action="navigate",
            parameters={"target": target},
            expected_duration=expected_duration
        )

    def validate(self) -> tuple[bool, Optional[str]]:
        target = self.parameters["target"]

        if isinstance(target, str):
            if target not in self.KNOWN_LOCATIONS:
                return False, f"Unknown location: {target}. Known locations: {self.KNOWN_LOCATIONS}"
        elif isinstance(target, list):
            if len(target) != 3:
                return False, "Coordinate target must be [x, y, theta]"
            if not all(isinstance(v, (int, float)) for v in target):
                return False, "Coordinates must be numeric"
        else:
            return False, "Target must be string (location name) or [x,y,theta] coordinates"

        return True, None


class Stop(ActionPrimitive):
    """Emergency stop - halt all motion immediately."""

    def __init__(self, expected_duration: float = 0.5):
        super().__init__(
            action="stop",
            parameters={},
            expected_duration=expected_duration
        )

    def validate(self) -> tuple[bool, Optional[str]]:
        return True, None  # No parameters to validate


class Follow(ActionPrimitive):
    """Track a moving target (person, object)."""

    VALID_TARGETS = ["person", "robot", "moving_object"]

    def __init__(self, target_class: str, distance: float = 2.0, expected_duration: float = 30.0):
        super().__init__(
            action="follow",
            parameters={"target_class": target_class, "distance": distance},
            expected_duration=expected_duration
        )

    def validate(self) -> tuple[bool, Optional[str]]:
        target_class = self.parameters["target_class"]
        distance = self.parameters["distance"]

        if target_class not in self.VALID_TARGETS:
            return False, f"Invalid target_class: {target_class}. Valid: {self.VALID_TARGETS}"

        if not isinstance(distance, (int, float)) or distance <= 0 or distance > 10:
            return False, "Distance must be 0-10 meters"

        return True, None


class DetectObject(ActionPrimitive):
    """Scan for objects matching a class using vision."""

    def __init__(self, object_class: str, max_results: int = 5, expected_duration: float = 3.0):
        super().__init__(
            action="detect_object",
            parameters={"object_class": object_class, "max_results": max_results},
            expected_duration=expected_duration
        )

    def validate(self) -> tuple[bool, Optional[str]]:
        object_class = self.parameters["object_class"]
        max_results = self.parameters["max_results"]

        if not isinstance(object_class, str) or len(object_class) == 0:
            return False, "object_class must be non-empty string"

        if not isinstance(max_results, int) or max_results < 1 or max_results > 20:
            return False, "max_results must be 1-20"

        return True, None


class ScanEnvironment(ActionPrimitive):
    """Perform 360° LIDAR scan to update occupancy map."""

    def __init__(self, expected_duration: float = 5.0):
        super().__init__(
            action="scan_environment",
            parameters={},
            expected_duration=expected_duration
        )

    def validate(self) -> tuple[bool, Optional[str]]:
        return True, None


class Grasp(ActionPrimitive):
    """Close gripper around detected object."""

    def __init__(self, object_id: str, force: float = 10.0, expected_duration: float = 4.0):
        super().__init__(
            action="grasp",
            parameters={"object_id": object_id, "force": force},
            expected_duration=expected_duration
        )

    def validate(self) -> tuple[bool, Optional[str]]:
        object_id = self.parameters["object_id"]
        force = self.parameters["force"]

        if not isinstance(object_id, str) or len(object_id) == 0:
            return False, "object_id must be non-empty string"

        if not isinstance(force, (int, float)) or force < 0 or force > 50:
            return False, "Force must be 0-50 Newtons"

        return True, None


class Release(ActionPrimitive):
    """Open gripper and drop held object."""

    def __init__(self, expected_duration: float = 2.0):
        super().__init__(
            action="release",
            parameters={},
            expected_duration=expected_duration
        )

    def validate(self) -> tuple[bool, Optional[str]]:
        return True, None


class OpenGripper(ActionPrimitive):
    """Set gripper to open position."""

    def __init__(self, width: float = 0.08, expected_duration: float = 1.5):
        super().__init__(
            action="open_gripper",
            parameters={"width": width},
            expected_duration=expected_duration
        )

    def validate(self) -> tuple[bool, Optional[str]]:
        width = self.parameters["width"]

        if not isinstance(width, (int, float)) or width < 0 or width > 0.15:
            return False, "Width must be 0-0.15 meters"

        return True, None


class CloseGripper(ActionPrimitive):
    """Set gripper to closed position."""

    def __init__(self, force: float = 5.0, expected_duration: float = 1.5):
        super().__init__(
            action="close_gripper",
            parameters={"force": force},
            expected_duration=expected_duration
        )

    def validate(self) -> tuple[bool, Optional[str]]:
        force = self.parameters["force"]

        if not isinstance(force, (int, float)) or force < 0 or force > 50:
            return False, "Force must be 0-50 Newtons"

        return True, None


class MoveArm(ActionPrimitive):
    """Move arm to predefined pose."""

    VALID_POSES = ["home", "ready", "stow", "neutral"]

    def __init__(self, pose_name: str, expected_duration: float = 3.0):
        super().__init__(
            action="move_arm",
            parameters={"pose_name": pose_name},
            expected_duration=expected_duration
        )

    def validate(self) -> tuple[bool, Optional[str]]:
        pose_name = self.parameters["pose_name"]

        if pose_name not in self.VALID_POSES:
            return False, f"Invalid pose_name: {pose_name}. Valid: {self.VALID_POSES}"

        return True, None


class Wait(ActionPrimitive):
    """Pause for specified duration."""

    def __init__(self, duration: float, expected_duration: Optional[float] = None):
        if expected_duration is None:
            expected_duration = duration

        super().__init__(
            action="wait",
            parameters={"duration": duration},
            expected_duration=expected_duration
        )

    def validate(self) -> tuple[bool, Optional[str]]:
        duration = self.parameters["duration"]

        if not isinstance(duration, (int, float)) or duration < 0 or duration > 60:
            return False, "Duration must be 0-60 seconds"

        return True, None


class Say(ActionPrimitive):
    """Text-to-speech output."""

    def __init__(self, text: str, expected_duration: float = 2.0):
        super().__init__(
            action="say",
            parameters={"text": text},
            expected_duration=expected_duration
        )

    def validate(self) -> tuple[bool, Optional[str]]:
        text = self.parameters["text"]

        if not isinstance(text, str) or len(text) == 0:
            return False, "text must be non-empty string"

        if len(text) > 500:
            return False, "text must be ≤500 characters"

        return True, None


# Primitive registry for factory pattern
PRIMITIVE_REGISTRY = {
    "navigate": Navigate,
    "stop": Stop,
    "follow": Follow,
    "detect_object": DetectObject,
    "scan_environment": ScanEnvironment,
    "grasp": Grasp,
    "release": Release,
    "open_gripper": OpenGripper,
    "close_gripper": CloseGripper,
    "move_arm": MoveArm,
    "wait": Wait,
    "say": Say,
}


def create_primitive_from_dict(action_dict: Dict[str, Any]) -> ActionPrimitive:
    """
    Factory function to create ActionPrimitive from dictionary.

    Args:
        action_dict: Dictionary with 'action', 'parameters', 'expected_duration'

    Returns:
        ActionPrimitive instance

    Raises:
        ValueError: If action name is unknown or parameters are invalid
    """
    action_name = action_dict.get("action")

    if action_name not in PRIMITIVE_REGISTRY:
        raise ValueError(f"Unknown action: {action_name}. Valid actions: {list(PRIMITIVE_REGISTRY.keys())}")

    primitive_class = PRIMITIVE_REGISTRY[action_name]
    parameters = action_dict.get("parameters", {})
    expected_duration = action_dict.get("expected_duration", 5.0)

    # Create instance with parameters
    try:
        if action_name == "navigate":
            primitive = primitive_class(parameters["target"], expected_duration)
        elif action_name == "stop":
            primitive = primitive_class(expected_duration)
        elif action_name == "follow":
            primitive = primitive_class(parameters["target_class"], parameters.get("distance", 2.0), expected_duration)
        elif action_name == "detect_object":
            primitive = primitive_class(parameters["object_class"], parameters.get("max_results", 5), expected_duration)
        elif action_name == "scan_environment":
            primitive = primitive_class(expected_duration)
        elif action_name == "grasp":
            primitive = primitive_class(parameters["object_id"], parameters.get("force", 10.0), expected_duration)
        elif action_name == "release":
            primitive = primitive_class(expected_duration)
        elif action_name == "open_gripper":
            primitive = primitive_class(parameters.get("width", 0.08), expected_duration)
        elif action_name == "close_gripper":
            primitive = primitive_class(parameters.get("force", 5.0), expected_duration)
        elif action_name == "move_arm":
            primitive = primitive_class(parameters["pose_name"], expected_duration)
        elif action_name == "wait":
            primitive = primitive_class(parameters["duration"], expected_duration)
        elif action_name == "say":
            primitive = primitive_class(parameters["text"], expected_duration)
        else:
            raise ValueError(f"Unhandled action: {action_name}")

    except KeyError as e:
        raise ValueError(f"Missing required parameter for {action_name}: {e}")

    return primitive


def validate_action_plan(plan: List[Dict[str, Any]]) -> tuple[bool, Optional[str]]:
    """
    Validate entire action plan.

    Args:
        plan: List of action dictionaries

    Returns:
        (is_valid, error_message)
    """
    if not isinstance(plan, list):
        return False, "Plan must be a list of actions"

    if len(plan) == 0:
        return False, "Plan cannot be empty"

    if len(plan) > 50:
        return False, "Plan too long (max 50 actions)"

    # Validate each action
    for i, action_dict in enumerate(plan):
        try:
            primitive = create_primitive_from_dict(action_dict)
            is_valid, error = primitive.validate()

            if not is_valid:
                return False, f"Action {i} ({action_dict['action']}): {error}"

        except ValueError as e:
            return False, f"Action {i}: {str(e)}"

    # Check preconditions (grasp requires detect_object before it)
    action_names = [a["action"] for a in plan]

    for i, action_name in enumerate(action_names):
        if action_name == "grasp":
            # Check if detect_object appears before grasp
            has_detection = "detect_object" in action_names[:i]
            if not has_detection:
                return False, f"Action {i} (grasp): requires detect_object before grasp"

    return True, None
