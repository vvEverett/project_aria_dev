#!/usr/bin/env python3
"""
Aria ROS 2 Publisher with Direct Display Integration

Extends aria_direct_display.py to publish both processed SLAM and RGB images
(including versions with and without distortion correction) to ROS 2 using SHM transport.

Features:
- Real-time stereo SLAM camera publishing (left/right)
- RGB camera publishing (supports both color and grayscale)
- Publishes both raw and undistorted versions of all images
- High-performance shared memory (SHM) transport
- OpenCV display integration from aria_direct_display
- Proper ROS 2 message conversion and timestamping

Topics Published:
- /aria/slam_left/raw - Raw left SLAM camera
- /aria/slam_left/undistorted - Undistorted left SLAM camera
- /aria/slam_right/raw - Raw right SLAM camera  
- /aria/slam_right/undistorted - Undistorted right SLAM camera
- /aria/rgb/raw - Raw RGB camera
- /aria/rgb/undistorted - Undistorted RGB camera

Prerequisites:
- rclpy
- sensor_msgs
- cv_bridge
- All dependencies from aria_direct_display
"""

import sys
import time
import os
import numpy as np
import cv2
import threading
from queue import LifoQueue, Empty, Queue

# ROS 2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import Header
    from cv_bridge import CvBridge
except ImportError:
    print("Error: ROS 2 dependencies not installed. Please install rclpy, sensor_msgs, cv_bridge")
    sys.exit(1)

# Aria SDK import
try:
    import aria.sdk as aria
except ImportError:
    print("Error: Project Aria SDK not installed. Please install projectaria_client_sdk")
    sys.exit(1)

# Import from aria_direct_display and aria_utils
from aria_direct_display import (
    AriaStreamingObserver, AriaDisplayManager,
    device_streaming_thread, STREAMING_PROFILE, ENABLE_RGB_STREAM, 
    ENABLE_UNDISTORTION, ROTATION_K, LEFT_OUTPUT_WIDTH, LEFT_OUTPUT_HEIGHT, 
    LEFT_FOCAL_LENGTH, RIGHT_OUTPUT_WIDTH, RIGHT_OUTPUT_HEIGHT, RIGHT_FOCAL_LENGTH,
    RGB_OUTPUT_WIDTH, RGB_OUTPUT_HEIGHT, RGB_FOCAL_LENGTH
)
from aria_utils import AriaUSBDeviceManager, CalibrationManager, setup_aria_sdk

# ======================= Configuration =======================
# ROS 2 QoS Profile for high-performance image transport
IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE
)
# =============================================================


class AriaROS2StreamingObserver(AriaStreamingObserver):
    """Extended observer that queues frames for both display and ROS 2 publishing."""
    
    def __init__(self, calibration_manager, display_queue, ros2_queue, running_flag):
        # Initialize with display queue for compatibility
        super().__init__(calibration_manager, display_queue, running_flag)
        self.ros2_queue = ros2_queue
        
    def on_image_received(self, image: np.ndarray, record):
        """Process incoming image data and store both raw and processed versions."""
        if record.camera_id == aria.CameraId.Slam1:
            camera_id_str = "left"
            self.frames_received_count_left += 1
        elif record.camera_id == aria.CameraId.Slam2:
            camera_id_str = "right"
            self.frames_received_count_right += 1
        elif record.camera_id == aria.CameraId.Rgb:
            camera_id_str = "rgb"
            self.frames_received_count_rgb += 1
        else:
            return  # Unknown camera
        
        # Process image (apply undistortion if enabled)
        processed_image = self.calibration_manager.process_image(image, camera_id_str)
        
        # Store both raw and processed data in both queues
        self._store_frame_data(record, image, processed_image, camera_id_str)
        
    def _store_frame_data(self, record, raw_image, processed_image, camera_id_str):
        """Store frame data for both display and ROS 2 publishing."""
        if not self.running_flag.is_set():
            return
            
        self.frame_count[camera_id_str] += 1
        
        if not self.first_frame_received[camera_id_str]:
            self.first_frame_received[camera_id_str] = True
            print(f"First {camera_id_str} frame received")
        
        # Create frame data for display queue (processed image)
        display_frame_data = {
            "camera_id": camera_id_str,
            "frame": processed_image,  # For display (undistorted if enabled)
            "timestamp": record.capture_timestamp_ns / 1e9
        }
        
        # Create frame data for ROS 2 queue (includes both raw and processed)
        ros2_frame_data = {
            "camera_id": camera_id_str,
            "frame": processed_image,  # For display (undistorted if enabled)
            "raw_frame": raw_image,    # For ROS publishing (always raw)
            "timestamp": record.capture_timestamp_ns / 1e9,
            "timestamp_ns": record.capture_timestamp_ns
        }
        
        # Try to put frame in display queue (non-blocking)
        try:
            self.frame_queue.put_nowait(display_frame_data)
        except:
            # Queue is full, discard oldest and add new
            try:
                self.frame_queue.get_nowait()
                self.frame_queue.put_nowait(display_frame_data)
            except:
                pass  # Skip if still can't queue
                
        # Try to put frame in ROS 2 queue (non-blocking)
        try:
            self.ros2_queue.put_nowait(ros2_frame_data)
        except:
            # Queue is full, discard oldest and add new
            try:
                self.ros2_queue.get_nowait()
                self.ros2_queue.put_nowait(ros2_frame_data)
            except:
                pass  # Skip if still can't queue


class AriaROS2Publisher(Node):
    """ROS 2 publisher for Aria camera streams with raw and undistorted versions."""
    
    def __init__(self, calibration_manager):
        super().__init__('aria_publisher')
        self.calibration_manager = calibration_manager
        self.cv_bridge = CvBridge()
        
        # Create publishers for all camera streams (raw and undistorted)
        self.image_publishers = {
            "slam_left_raw": self.create_publisher(Image, '/aria/slam_left/raw', IMAGE_QOS),
            "slam_left_undistorted": self.create_publisher(Image, '/aria/slam_left/undistorted', IMAGE_QOS),
            "slam_right_raw": self.create_publisher(Image, '/aria/slam_right/raw', IMAGE_QOS),
            "slam_right_undistorted": self.create_publisher(Image, '/aria/slam_right/undistorted', IMAGE_QOS),
        }
        
        if ENABLE_RGB_STREAM:
            self.image_publishers.update({
                "rgb_raw": self.create_publisher(Image, '/aria/rgb/raw', IMAGE_QOS),
                "rgb_undistorted": self.create_publisher(Image, '/aria/rgb/undistorted', IMAGE_QOS),
            })
        
        # Statistics
        self.published_count = {"left": 0, "right": 0, "rgb": 0}
        self.last_stats_time = time.time()
        
        self.get_logger().info("Aria ROS 2 Publisher initialized")
        self.get_logger().info("Publishing with SHM transport")
    
    def publish_frame(self, frame_data):
        """Publish both raw and processed versions of a frame."""
        camera_id = frame_data["camera_id"]
        raw_frame = frame_data["raw_frame"]
        processed_frame = frame_data["frame"]
        timestamp_ns = frame_data["timestamp_ns"]
        
        try:
            # Create ROS header with proper timestamp
            header = Header()
            header.stamp.sec = int(timestamp_ns // 1_000_000_000)
            header.stamp.nanosec = int(timestamp_ns % 1_000_000_000)
            header.frame_id = f"aria_{camera_id}_camera"
            
            # OPTIMIZATION: Process both images together to reduce overhead
            # Rotate images for proper orientation (use optimized rotation)
            if ROTATION_K != 0:
                raw_rotated = np.rot90(raw_frame, k=ROTATION_K)
                processed_rotated = np.rot90(processed_frame, k=ROTATION_K)
            else:
                raw_rotated = raw_frame
                processed_rotated = processed_frame
            
            # OPTIMIZATION: Only ensure contiguous if needed
            if not raw_rotated.flags['C_CONTIGUOUS']:
                raw_rotated = np.ascontiguousarray(raw_rotated)
            if not processed_rotated.flags['C_CONTIGUOUS']:
                processed_rotated = np.ascontiguousarray(processed_rotated)
            
            # Determine encoding once for both images
            if raw_rotated.ndim == 2:
                encoding = "mono8"
            elif raw_rotated.ndim == 3 and raw_rotated.shape[2] == 3:
                encoding = "rgb8"  # Aria streams in RGB format
            else:
                encoding = "mono8"
                raw_rotated = raw_rotated.squeeze()
                processed_rotated = processed_rotated.squeeze()
            
            # OPTIMIZATION: Create messages more efficiently
            raw_msg = self.cv_bridge.cv2_to_imgmsg(raw_rotated, encoding=encoding)
            raw_msg.header = header
            
            # Reuse header for second message
            undistorted_msg = self.cv_bridge.cv2_to_imgmsg(processed_rotated, encoding=encoding)
            undistorted_msg.header = header
            
            # Select appropriate publishers
            if camera_id == "left":
                self.image_publishers["slam_left_raw"].publish(raw_msg)
                self.image_publishers["slam_left_undistorted"].publish(undistorted_msg)
            elif camera_id == "right":
                self.image_publishers["slam_right_raw"].publish(raw_msg)
                self.image_publishers["slam_right_undistorted"].publish(undistorted_msg)
            elif camera_id == "rgb" and ENABLE_RGB_STREAM:
                self.image_publishers["rgb_raw"].publish(raw_msg)
                self.image_publishers["rgb_undistorted"].publish(undistorted_msg)
            
            self.published_count[camera_id] += 1
            
        except Exception as e:
            self.get_logger().error(f"Error publishing {camera_id} frame: {e}")
    
    def print_statistics(self):
        """Print publishing statistics."""
        current_time = time.time()
        if current_time - self.last_stats_time >= 5.0:
            time_diff = current_time - self.last_stats_time
            
            left_fps = self.published_count["left"] / time_diff
            right_fps = self.published_count["right"] / time_diff  
            rgb_fps = self.published_count["rgb"] / time_diff
            
            timestamp = time.strftime("%H:%M:%S", time.localtime(current_time))
            self.get_logger().info(
                f"[{timestamp}] Publishing FPS: Left={left_fps:.1f}, Right={right_fps:.1f}, RGB={rgb_fps:.1f}"
            )
            
            # Reset counters
            self.published_count = {"left": 0, "right": 0, "rgb": 0}
            self.last_stats_time = current_time


def ros2_publisher_thread(ros2_publisher, frame_queue, running_flag):
    """Background thread for ROS 2 publishing."""
    try:
        print("ROS 2 publisher thread starting...")
        
        while running_flag.is_set() and rclpy.ok():
            frames_processed = 0
            
            # Process multiple frames in a batch for better performance
            while frames_processed < 10:  # Process up to 10 frames per batch
                try:
                    frame_data = frame_queue.get(timeout=0.01)  # Shorter timeout
                    ros2_publisher.publish_frame(frame_data)
                    frames_processed += 1
                except Empty:
                    break  # No more frames to process
                except Exception as e:
                    ros2_publisher.get_logger().error(f"Error in publisher thread: {e}")
                    break
            
            # Print statistics (less frequently to avoid spam)
            ros2_publisher.print_statistics()
            
            # Small sleep to prevent excessive CPU usage
            if frames_processed == 0:
                time.sleep(0.01)
                
        print("ROS 2 publisher thread stopping...")
        
    except Exception as e:
        print(f"Error in ROS 2 publisher thread: {e}")
        import traceback
        traceback.print_exc()


def enhanced_device_streaming_thread(device_manager, calibration_manager, 
                                   display_queue, ros2_queue, running_flag):
    """Enhanced streaming thread that feeds both display and ROS 2 queues."""
    observer = None
    
    try:
        print("Enhanced device streaming thread starting...")
        
        # Initialize observer with both queues
        observer = AriaROS2StreamingObserver(calibration_manager, display_queue, ros2_queue, running_flag)
        
        # Start streaming
        device_manager.start_streaming()
        device_manager.streaming_client.set_streaming_client_observer(observer)
        device_manager.streaming_client.subscribe()
        
        image_type_str = "undistorted" if ENABLE_UNDISTORTION else "raw"
        stream_types = "SLAM"
        if ENABLE_RGB_STREAM:
            stream_types += " + RGB"
        print(f"Enhanced streaming: Subscribed to {stream_types} data stream via USB ({image_type_str} images)")
        
        # Statistics tracking
        last_stats_time = time.time()
        last_frame_count = {"left": 0, "right": 0, "rgb": 0}
        
        while running_flag.is_set():
            current_time = time.time()
            
            # Print statistics every 5 seconds
            if current_time - last_stats_time > 5.0:
                time_diff = current_time - last_stats_time
                left_fps = (observer.frame_count["left"] - last_frame_count["left"]) / time_diff
                right_fps = (observer.frame_count["right"] - last_frame_count["right"]) / time_diff
                rgb_fps = (observer.frame_count["rgb"] - last_frame_count["rgb"]) / time_diff
                
                timestamp = time.strftime("%H:%M:%S", time.localtime(current_time))
                print(f"[{timestamp}] Streaming FPS: Left={left_fps:.1f}, Right={right_fps:.1f}, RGB={rgb_fps:.1f} | "
                      f"Total frames: L={observer.frame_count['left']} R={observer.frame_count['right']} RGB={observer.frame_count['rgb']}")
                
                last_frame_count = observer.frame_count.copy()
                last_stats_time = current_time
            
            time.sleep(0.1)
            
    except Exception as e:
        print(f"Error in enhanced streaming thread: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Enhanced streaming thread stopping...")
        if observer and device_manager.streaming_client:
            try:
                device_manager.streaming_client.unsubscribe()
            except Exception as e:
                print(f"Error unsubscribing in enhanced streaming thread: {e}")


def main():
    """Main application entry point with ROS 2 integration."""
    print("Aria ROS 2 Publisher with Direct Display - Real-time camera stream publishing")
    print("Controls: 'q'=quit, 's'=save images, 'u'=toggle undistortion, ESC=quit")
    
    # Initialize ROS 2
    rclpy.init()
    
    setup_aria_sdk()
    
    # Initialize managers
    device_manager = AriaUSBDeviceManager(STREAMING_PROFILE, ENABLE_RGB_STREAM)
    display_manager = AriaDisplayManager()
    calibration_manager = None
    streaming_thread = None
    ros2_thread = None
    ros2_publisher = None
    
    # Create separate queues for display and ROS 2
    display_queue = LifoQueue(maxsize=10) # LifoQueue: Last In First Out (LIFO)
    ros2_queue = Queue(maxsize=10)  # Queue: First In First Out (FIFO)
    running_flag = threading.Event()
    running_flag.set()
    
    try:
        # Connect and setup device
        device_manager.connect()
        device_manager.stop_existing_sessions()

        # Handle calibration
        device_calibration = None
        if ENABLE_UNDISTORTION:
            device_calibration = device_manager.get_calibration()
            if not device_calibration:
                print("Using raw images due to calibration failure.")

        # Initialize calibration manager
        calibration_manager = CalibrationManager(
            device_calibration, 
            ENABLE_UNDISTORTION,
            LEFT_OUTPUT_WIDTH, LEFT_OUTPUT_HEIGHT, LEFT_FOCAL_LENGTH,
            RIGHT_OUTPUT_WIDTH, RIGHT_OUTPUT_HEIGHT, RIGHT_FOCAL_LENGTH,
            RGB_OUTPUT_WIDTH, RGB_OUTPUT_HEIGHT, RGB_FOCAL_LENGTH
        )
        
        # Initialize ROS 2 publisher
        ros2_publisher = AriaROS2Publisher(calibration_manager)

        # Start background streaming thread
        streaming_thread = threading.Thread(
            target=enhanced_device_streaming_thread,
            args=(device_manager, calibration_manager, display_queue, ros2_queue, running_flag)
        )
        streaming_thread.daemon = True
        streaming_thread.start()
        
        # Start ROS 2 publisher thread
        ros2_thread = threading.Thread(
            target=ros2_publisher_thread,
            args=(ros2_publisher, ros2_queue, running_flag)
        )
        ros2_thread.daemon = True
        ros2_thread.start()
        
        print("Display and ROS 2 publishing started. Use controls to interact with the application.")
        
        # Initialize display variables
        current_frames = {"left": None, "right": None, "rgb": None}
        
        while True:
            # Process frames from display queue (non-blocking)
            try:
                while True:
                    frame_data = display_queue.get_nowait()
                    camera_id = frame_data["camera_id"]
                    current_frames[camera_id] = frame_data["frame"]
                    
            except Empty:
                pass  # No more frames in queue
            
            # Display frames (keeps existing processing pipeline)
            display_manager.display_frames(current_frames)
            
            # Handle keyboard input
            key = cv2.waitKey(10) & 0xFF
            if key == ord('q') or key == 27:  # 'q' or ESC
                break
            elif key == ord('s'):
                display_manager.save_current_frames(current_frames)
            elif key == ord('u'):
                # Toggle undistortion
                enabled = display_manager.toggle_undistortion()
                calibration_manager.use_undistorted = enabled
                print(f"Undistortion {'enabled' if enabled else 'disabled'}")
            
            # Spin ROS 2 node (non-blocking)
            rclpy.spin_once(ros2_publisher, timeout_sec=0)
            
    except KeyboardInterrupt:
        print("\nUser interrupted (Ctrl+C)")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("=" * 50)
        print("Shutting down, cleaning up resources...")
        
        # Stop background threads
        running_flag.clear()
        
        if streaming_thread and streaming_thread.is_alive():
            print("Waiting for streaming thread to stop...")
            streaming_thread.join(timeout=3.0)
            
        if ros2_thread and ros2_thread.is_alive():
            print("Waiting for ROS 2 thread to stop...")
            ros2_thread.join(timeout=3.0)
        
        # Cleanup resources
        display_manager.cleanup()
        device_manager.cleanup()
        
        if ros2_publisher:
            ros2_publisher.destroy_node()
        rclpy.shutdown()
        
        time.sleep(0.5)
        print("Cleanup complete.")
        print("=" * 50)


if __name__ == "__main__":
    main()