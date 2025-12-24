#!/usr/bin/env python3
"""
Voice Command Node using OpenAI Whisper for ROS 2

This node captures audio from the microphone, transcribes it using OpenAI Whisper,
and publishes the transcription to the /voice_commands topic.

Author: Module 4 - Vision-Language-Action (VLA)
License: MIT
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import sounddevice as sd
import numpy as np
import threading
import queue
from typing import Optional


class VoiceCommandNode(Node):
    """
    ROS 2 node for voice command recognition using OpenAI Whisper.

    Parameters:
        model_size (str): Whisper model size ('base' or 'small')
        confidence_threshold (float): Minimum confidence for publishing (0.0-1.0)
        sample_rate (int): Audio sample rate in Hz (16000 recommended)
        device_index (int): Microphone device index (-1 for default)
        recording_duration (float): Duration of each recording in seconds

    Topics:
        Publishers:
            /voice_commands (std_msgs/String): Transcribed voice commands
    """

    def __init__(self):
        super().__init__('voice_command_node')

        # Declare parameters
        self.declare_parameter('model_size', 'base')
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('device_index', -1)
        self.declare_parameter('recording_duration', 3.0)

        # Get parameters
        model_size = self.get_parameter('model_size').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.device_index = self.get_parameter('device_index').value
        self.recording_duration = self.get_parameter('recording_duration').value

        # Load Whisper model
        self.get_logger().info(f'Loading Whisper model: {model_size}...')
        self.model = whisper.load_model(model_size)
        self.get_logger().info('Whisper model loaded successfully')

        # Create publisher
        self.publisher = self.create_publisher(String, 'voice_commands', 10)

        # Audio processing queue
        self.audio_queue = queue.Queue()

        # Start audio capture thread
        self.recording = True
        self.audio_thread = threading.Thread(target=self._audio_capture_loop, daemon=True)
        self.audio_thread.start()

        # Start transcription timer (process queue every 0.5 seconds)
        self.timer = self.create_timer(0.5, self._process_audio_queue)

        self.get_logger().info(
            f'Voice command node started with model={model_size}, '
            f'confidence_threshold={self.confidence_threshold}, '
            f'sample_rate={self.sample_rate}'
        )

    def _audio_capture_loop(self) -> None:
        """
        Continuously capture audio from microphone and add to queue.
        Runs in separate thread to avoid blocking ROS 2 callbacks.
        """
        device = self.device_index if self.device_index >= 0 else None

        self.get_logger().info('Starting audio capture (press Ctrl+C to stop)...')

        while self.recording:
            try:
                # Record audio
                audio_data = sd.rec(
                    int(self.recording_duration * self.sample_rate),
                    samplerate=self.sample_rate,
                    channels=1,
                    dtype='float32',
                    device=device
                )
                sd.wait()  # Wait for recording to complete

                # Convert to mono and normalize
                audio_data = audio_data.flatten()

                # Check if audio contains speech (simple energy threshold)
                energy = np.sqrt(np.mean(audio_data ** 2))
                if energy > 0.01:  # Noise gate threshold
                    self.audio_queue.put(audio_data)
                    self.get_logger().debug(f'Audio captured (energy={energy:.4f})')

            except Exception as e:
                self.get_logger().error(f'Error capturing audio: {str(e)}')

    def _process_audio_queue(self) -> None:
        """
        Process audio from queue and publish transcriptions.
        Called by ROS 2 timer callback.
        """
        try:
            # Process all queued audio
            while not self.audio_queue.empty():
                audio_data = self.audio_queue.get_nowait()

                # Transcribe with Whisper
                result = self.model.transcribe(
                    audio_data,
                    language='en',
                    fp16=False  # Use FP32 for CPU compatibility
                )

                text = result['text'].strip()

                # Get confidence score (average log probability)
                # Note: Whisper doesn't provide direct confidence, we estimate from logprobs
                segments = result.get('segments', [])
                if segments:
                    avg_logprob = np.mean([seg.get('avg_logprob', -1.0) for seg in segments])
                    # Convert log probability to approximate confidence (0-1)
                    confidence = np.exp(avg_logprob)
                else:
                    confidence = 0.0

                # Publish if above confidence threshold
                if text and confidence >= self.confidence_threshold:
                    msg = String()
                    msg.data = text
                    self.publisher.publish(msg)
                    self.get_logger().info(
                        f'Published: "{text}" (confidence={confidence:.2f})'
                    )
                elif text:
                    self.get_logger().warn(
                        f'Low confidence: "{text}" (confidence={confidence:.2f} < '
                        f'{self.confidence_threshold})'
                    )

        except queue.Empty:
            pass
        except Exception as e:
            self.get_logger().error(f'Error processing audio: {str(e)}')

    def destroy_node(self) -> None:
        """Clean up resources on shutdown."""
        self.recording = False
        if self.audio_thread.is_alive():
            self.audio_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    """Main entry point for the voice command node."""
    rclpy.init(args=args)

    try:
        node = VoiceCommandNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
