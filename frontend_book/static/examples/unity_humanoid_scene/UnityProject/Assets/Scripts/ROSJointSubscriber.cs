using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System.Collections.Generic;

/// <summary>
/// ROSJointSubscriber - Subscribes to ROS 2 joint states and updates Unity articulation bodies
///
/// This script demonstrates Unity-ROS 2 integration for robot joint control.
/// Attach this component to the root GameObject of your robot hierarchy.
///
/// Prerequisites:
/// - ROS-TCP-Connector package installed (com.unity.robotics.ros-tcp-connector v0.7.0+)
/// - Robot model imported with ArticulationBody components on each joint
/// - ROS 2 TCP endpoint running: ros2 run ros_tcp_endpoint default_server_endpoint
///
/// ROS 2 Topic Input:
/// - Topic: /joint_states
/// - Message Type: sensor_msgs/JointState
/// - Fields: name (joint names), position (radians), velocity (rad/s), effort (Nm)
///
/// Unity Requirements:
/// - Each robot link must have an ArticulationBody component
/// - ArticulationBody joint names must match URDF joint names
/// - Root link (base_link) should be ArticulationBody with immovable=true
///
/// Usage:
/// 1. Attach this script to robot root GameObject (e.g., simple_humanoid)
/// 2. Populate articulationBodies array in Inspector with robot's ArticulationBody components
/// 3. Verify joint names match URDF (left_shoulder_joint, left_elbow_joint, etc.)
/// 4. Start ROS 2 TCP endpoint in terminal
/// 5. Press Play in Unity
/// 6. Publish joint commands from ROS 2:
///    ros2 topic pub /joint_states sensor_msgs/msg/JointState \
///    '{name: ["left_shoulder_joint"], position: [0.5]}' --once
///
/// Author: AI-Driven Robotics Textbook
/// Module: 2 - The Digital Twin (Gazebo & Unity)
/// Chapter: 2 - Unity Rendering
/// Date: 2025-12-19
/// </summary>
public class ROSJointSubscriber : MonoBehaviour
{
    [Header("Robot Configuration")]
    [Tooltip("Array of ArticulationBody components for all robot joints")]
    public ArticulationBody[] articulationBodies;

    [Header("ROS 2 Configuration")]
    [Tooltip("ROS 2 topic name for joint states")]
    public string topicName = "/joint_states";

    [Header("Control Settings")]
    [Tooltip("Maximum joint velocity in rad/s (safety limit)")]
    public float maxJointVelocity = 2.0f;

    [Tooltip("Enable debug logging for joint updates")]
    public bool debugMode = false;

    // Private members
    private ROSConnection ros;
    private Dictionary<string, ArticulationBody> jointMap;
    private int messagesReceived = 0;

    void Start()
    {
        // Get ROS connection instance
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>(topicName, OnJointStateReceived);

        // Build joint name → ArticulationBody mapping
        BuildJointMap();

        Debug.Log($"ROSJointSubscriber: Subscribed to '{topicName}' - Monitoring {jointMap.Count} joints");
    }

    /// <summary>
    /// Build a dictionary mapping joint names to ArticulationBody components
    /// </summary>
    void BuildJointMap()
    {
        jointMap = new Dictionary<string, ArticulationBody>();

        if (articulationBodies == null || articulationBodies.Length == 0)
        {
            Debug.LogWarning("ROSJointSubscriber: No ArticulationBodies assigned in Inspector! Auto-detecting...");
            articulationBodies = GetComponentsInChildren<ArticulationBody>();
        }

        foreach (var articulation in articulationBodies)
        {
            if (articulation != null)
            {
                // Use GameObject name as joint name (should match URDF joint name)
                string jointName = articulation.gameObject.name;

                // Skip root link (immovable base)
                if (articulation.isRoot)
                {
                    if (debugMode)
                        Debug.Log($"ROSJointSubscriber: Skipping root link '{jointName}'");
                    continue;
                }

                jointMap[jointName] = articulation;

                if (debugMode)
                    Debug.Log($"ROSJointSubscriber: Mapped joint '{jointName}' → ArticulationBody");
            }
        }

        if (jointMap.Count == 0)
        {
            Debug.LogError("ROSJointSubscriber: No valid joints found! Check ArticulationBody setup.");
        }
    }

    /// <summary>
    /// Callback for ROS 2 JointState messages
    /// </summary>
    void OnJointStateReceived(JointStateMsg msg)
    {
        messagesReceived++;

        // Validate message
        if (msg.name == null || msg.name.Length == 0)
        {
            Debug.LogWarning("ROSJointSubscriber: Received empty JointState message");
            return;
        }

        // Process each joint in the message
        for (int i = 0; i < msg.name.Length; i++)
        {
            string jointName = msg.name[i];

            // Check if this joint exists in our robot
            if (!jointMap.ContainsKey(jointName))
            {
                if (debugMode)
                    Debug.LogWarning($"ROSJointSubscriber: Unknown joint '{jointName}' - ignoring");
                continue;
            }

            ArticulationBody joint = jointMap[jointName];

            // Update joint position (if provided)
            if (msg.position != null && i < msg.position.Length)
            {
                float targetPosition = (float)msg.position[i];  // Radians
                SetJointPosition(joint, targetPosition);

                if (debugMode)
                    Debug.Log($"ROSJointSubscriber: Set '{jointName}' position = {targetPosition:F3} rad");
            }

            // Update joint velocity (if provided)
            if (msg.velocity != null && i < msg.velocity.Length)
            {
                float targetVelocity = (float)msg.velocity[i];  // Rad/s

                // Apply safety limit
                targetVelocity = Mathf.Clamp(targetVelocity, -maxJointVelocity, maxJointVelocity);

                SetJointVelocity(joint, targetVelocity);

                if (debugMode)
                    Debug.Log($"ROSJointSubscriber: Set '{jointName}' velocity = {targetVelocity:F3} rad/s");
            }
        }
    }

    /// <summary>
    /// Set ArticulationBody joint position (revolute or prismatic)
    /// </summary>
    void SetJointPosition(ArticulationBody joint, float position)
    {
        // Get current joint drive settings
        ArticulationDrive drive = joint.xDrive;

        // Set target position
        drive.target = position * Mathf.Rad2Deg;  // Convert radians to degrees for Unity

        // Apply updated drive
        joint.xDrive = drive;
    }

    /// <summary>
    /// Set ArticulationBody joint velocity (revolute or prismatic)
    /// </summary>
    void SetJointVelocity(ArticulationBody joint, float velocity)
    {
        // Get current joint drive settings
        ArticulationDrive drive = joint.xDrive;

        // Set target velocity
        drive.targetVelocity = velocity * Mathf.Rad2Deg;  // Convert rad/s to deg/s for Unity

        // Apply updated drive
        joint.xDrive = drive;
    }

    /// <summary>
    /// Display status in Unity Editor (for debugging)
    /// </summary>
    void OnGUI()
    {
        if (Application.isPlaying)
        {
            GUI.Label(new Rect(10, 70, 400, 20), $"Joint Subscriber: {topicName}");
            GUI.Label(new Rect(10, 90, 400, 20), $"Joints Mapped: {jointMap.Count}");
            GUI.Label(new Rect(10, 110, 400, 20), $"Messages Received: {messagesReceived}");
            GUI.Label(new Rect(10, 130, 400, 20), $"ROS Connected: {ros.HasConnectionThread}");
        }
    }
}
