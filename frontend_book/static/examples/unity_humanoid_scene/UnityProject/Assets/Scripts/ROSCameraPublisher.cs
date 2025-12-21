using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

/// <summary>
/// ROSCameraPublisher - Captures camera images and publishes to ROS 2 topic
///
/// This script demonstrates Unity-ROS 2 integration for vision systems.
/// Attach this component to a Camera GameObject to publish images to ROS 2.
///
/// Prerequisites:
/// - ROS-TCP-Connector package installed (com.unity.robotics.ros-tcp-connector v0.7.0+)
/// - ROS 2 TCP endpoint running: ros2 run ros_tcp_endpoint default_server_endpoint
///
/// ROS 2 Topic Output:
/// - Topic: /camera/rgb/image_raw
/// - Message Type: sensor_msgs/Image
/// - Encoding: RGB8 (24-bit color, 8 bits per channel)
/// - Frame ID: camera_link
///
/// Usage:
/// 1. Attach this script to Main Camera in Unity scene
/// 2. Set imageWidth and imageHeight in Inspector (default: 640x480)
/// 3. Set publishRate in Inspector (default: 10 Hz)
/// 4. Start ROS 2 TCP endpoint in terminal
/// 5. Press Play in Unity
/// 6. View images in RViz: rviz2, add Image display, set topic to /camera/rgb/image_raw
///
/// Author: AI-Driven Robotics Textbook
/// Module: 2 - The Digital Twin (Gazebo & Unity)
/// Chapter: 2 - Unity Rendering
/// Date: 2025-12-19
/// </summary>
public class ROSCameraPublisher : MonoBehaviour
{
    [Header("Camera Configuration")]
    [Tooltip("Image width in pixels")]
    public int imageWidth = 640;

    [Tooltip("Image height in pixels")]
    public int imageHeight = 480;

    [Header("ROS 2 Configuration")]
    [Tooltip("ROS 2 topic name for camera images")]
    public string topicName = "/camera/rgb/image_raw";

    [Tooltip("Frame ID for TF transforms (should match URDF camera link name)")]
    public string frameId = "camera_link";

    [Tooltip("Publish rate in Hz (images per second)")]
    public float publishRate = 10.0f;

    // Private members
    private ROSConnection ros;
    private Camera cam;
    private RenderTexture renderTexture;
    private Texture2D texture2D;
    private float publishInterval;
    private float timeSinceLastPublish;

    void Start()
    {
        // Get ROS connection instance
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);

        // Get Camera component
        cam = GetComponent<Camera>();
        if (cam == null)
        {
            Debug.LogError("ROSCameraPublisher: No Camera component found on this GameObject!");
            enabled = false;
            return;
        }

        // Create RenderTexture for camera output
        renderTexture = new RenderTexture(imageWidth, imageHeight, 24);
        cam.targetTexture = renderTexture;

        // Create Texture2D for reading pixel data
        texture2D = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);

        // Calculate publish interval
        publishInterval = 1.0f / publishRate;
        timeSinceLastPublish = 0.0f;

        Debug.Log($"ROSCameraPublisher: Initialized - Publishing to '{topicName}' at {publishRate} Hz");
    }

    void Update()
    {
        // Publish at specified rate
        timeSinceLastPublish += Time.deltaTime;

        if (timeSinceLastPublish >= publishInterval)
        {
            PublishCameraImage();
            timeSinceLastPublish = 0.0f;
        }
    }

    void PublishCameraImage()
    {
        // Read pixels from RenderTexture
        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        texture2D.Apply();
        RenderTexture.active = null;

        // Get raw pixel data (RGB24 format: R, G, B, R, G, B, ...)
        byte[] imageData = texture2D.GetRawTextureData();

        // Create ROS Image message
        ImageMsg imageMsg = new ImageMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time - (int)Time.time) * 1e9)
                },
                frame_id = frameId
            },
            height = (uint)imageHeight,
            width = (uint)imageWidth,
            encoding = "rgb8",  // 8-bit RGB encoding
            is_bigendian = 0,   // Little-endian byte order
            step = (uint)(imageWidth * 3),  // Row stride in bytes (width * 3 channels)
            data = imageData
        };

        // Publish to ROS 2 topic
        ros.Publish(topicName, imageMsg);
    }

    void OnDestroy()
    {
        // Clean up resources
        if (renderTexture != null)
        {
            renderTexture.Release();
        }

        if (texture2D != null)
        {
            Destroy(texture2D);
        }
    }

    /// <summary>
    /// Display status in Unity Editor (for debugging)
    /// </summary>
    void OnGUI()
    {
        if (Application.isPlaying)
        {
            GUI.Label(new Rect(10, 10, 400, 20), $"Camera Publishing: {topicName} @ {publishRate} Hz");
            GUI.Label(new Rect(10, 30, 400, 20), $"Resolution: {imageWidth}x{imageHeight}");
            GUI.Label(new Rect(10, 50, 400, 20), $"ROS Connected: {ros.HasConnectionThread}");
        }
    }
}
