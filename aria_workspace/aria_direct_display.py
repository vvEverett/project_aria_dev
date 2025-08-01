#!/usr/bin/env python3
"""
Aria Direct Display Receiver

Connects directly to Project Aria device via USB and displays real-time camera streams.
No ZMQ or ROS dependencies - pure Aria SDK integration with OpenCV display.

Features:
- Direct USB connection to Aria device
- Real-time stereo SLAM camera display (left/right side-by-side)
- RGB camera display (supports both color and grayscale)
- Optional image undistortion using device calibration
- Automatic image rotation for proper orientation
- Frame statistics and device status monitoring
- Image saving functionality with timestamped filenames

Controls:
- 'q': Quit application
- 's': Save current frames to disk
- 'u': Toggle undistortion on/off
- ESC: Quit application

Prerequisites:
- projectaria_client_sdk
- opencv-python
- numpy
"""

import sys
import time
import numpy as np
import cv2
import os
import threading
from queue import LifoQueue, Empty

try:
    import aria.sdk as aria
except ImportError:
    print("Error: Project Aria SDK not installed. Please install projectaria_client_sdk")
    sys.exit(1)

from aria_utils import (
    AriaDeviceManager, AriaUSBDeviceManager, CalibrationManager, BaseAriaStreamObserver, 
    AriaDisplayManager, setup_aria_sdk
)

# ======================= Configuration =======================
STREAMING_PROFILE = "profile12"  # USB streaming profile
ROTATION_K = -1  # np.rot90 k parameter: -1=counterclockwise 90°, 1=clockwise 90°
SAVE_BASE_DIR = "/home/developer/aria_workspace"  # Base directory for saving images
ENABLE_RGB_STREAM = True  # Set to True to enable RGB camera stream
ENABLE_UNDISTORTION = True  # Set to True to apply undistortion by default
ENABLE_STATISTICS = True  # Set to True to show frame statistics
ENABLE_DISPLAY = True  # Set to False to disable OpenCV display windows (headless mode)
DISPLAY_SCALE = 1.0  # Scale factor for display windows (1.0 = original size)

# Undistortion configuration
LEFT_OUTPUT_WIDTH = 640
LEFT_OUTPUT_HEIGHT = 480
LEFT_FOCAL_LENGTH = None  # None = use original from calibration

RIGHT_OUTPUT_WIDTH = 640
RIGHT_OUTPUT_HEIGHT = 480
RIGHT_FOCAL_LENGTH = None

RGB_OUTPUT_WIDTH = 512 # Default RGB camera resolution is 1408*1408, but we use 512x512
RGB_OUTPUT_HEIGHT = 512
RGB_FOCAL_LENGTH = 222.3 # Focal length for 512x512 RGB camera, default is 611
# =============================================================

class AriaStreamingObserver(BaseAriaStreamObserver):
    """Observer for receiving frames from Aria SDK and queuing them for display."""
    
    def __init__(self, calibration_manager, frame_queue, running_flag):
        super().__init__(calibration_manager)
        self.frame_queue = frame_queue
        self.running_flag = running_flag
        self.frame_count = {"left": 0, "right": 0, "rgb": 0}
        self.first_frame_received = {"left": False, "right": False, "rgb": False}
        
    def _store_frame_data(self, record, processed_image, camera_id_str):
        """Store frame data in queue for background processing."""
        if not self.running_flag.is_set():
            return
            
        self.frame_count[camera_id_str] += 1
        
        if not self.first_frame_received[camera_id_str]:
            self.first_frame_received[camera_id_str] = True
            print(f"First {camera_id_str} frame received")
        
        # Create frame data for queue
        frame_data = {
            "camera_id": camera_id_str,
            "frame": processed_image,
            "timestamp": record.capture_timestamp_ns / 1e9
        }
        
        # Try to put frame in queue (non-blocking)
        try:
            self.frame_queue.put_nowait(frame_data)
        except:
            # Queue is full, discard oldest and add new
            try:
                self.frame_queue.get_nowait()
                self.frame_queue.put_nowait(frame_data)
            except:
                pass  # Skip if still can't queue


def device_streaming_thread(device_manager, calibration_manager, frame_queue, running_flag):
    """Background thread for device streaming and frame capture."""
    observer = None
    
    try:
        print("Device streaming thread starting...")
        
        # Initialize calibration manager and observer
        observer = AriaStreamingObserver(calibration_manager, frame_queue, running_flag)

        # Start streaming
        device_manager.start_streaming()
        device_manager.streaming_client.set_streaming_client_observer(observer)
        device_manager.streaming_client.subscribe()
        
        image_type_str = "undistorted" if ENABLE_UNDISTORTION else "raw"
        stream_types = "SLAM"
        if ENABLE_RGB_STREAM:
            stream_types += " + RGB"
        print(f"Device streaming thread: Subscribed to {stream_types} data stream via USB ({image_type_str} images)")
        
        # Statistics tracking
        last_stats_time = time.time()
        last_frame_count = {"left": 0, "right": 0, "rgb": 0}
        
        while running_flag.is_set():
            current_time = time.time()
            
            # Print statistics every 5 seconds with FPS calculation
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
            
            time.sleep(0.1)  # Small delay to prevent excessive CPU usage
            
    except Exception as e:
        print(f"Error in device streaming thread: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Device streaming thread stopping...")
        if observer and device_manager.streaming_client:
            try:
                device_manager.streaming_client.unsubscribe()
            except Exception as e:
                print(f"Error unsubscribing in streaming thread: {e}")


def main():
    """Main application entry point."""
    print("Aria Direct Display - Real-time camera stream visualization (Threaded)")
    if ENABLE_DISPLAY:
        print("Controls: 'q'=quit, 's'=save images, 'u'=toggle undistortion, ESC=quit")
    else:
        print("Running in headless mode (no display). Press Ctrl+C to quit.")
    
    setup_aria_sdk()
    
    # Initialize managers
    device_manager = AriaUSBDeviceManager(STREAMING_PROFILE, ENABLE_RGB_STREAM)
    display_manager = AriaDisplayManager(
        display_scale=DISPLAY_SCALE,
        save_base_dir=SAVE_BASE_DIR,
        rotation_k=ROTATION_K,
        enable_rgb_stream=ENABLE_RGB_STREAM,
        enable_undistortion=ENABLE_UNDISTORTION
    )
    calibration_manager = None
    streaming_thread = None
    
    # Create frame queue and threading controls
    frame_queue = LifoQueue(maxsize=10)  # LIFO queue for latest frames
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

        # Start background streaming thread
        streaming_thread = threading.Thread(
            target=device_streaming_thread,
            args=(device_manager, calibration_manager, frame_queue, running_flag)
        )
        streaming_thread.daemon = True
        streaming_thread.start()
        
        print("Display started. Use controls to interact with the application.")
        
        # Initialize display variables
        current_frames = {"left": None, "right": None, "rgb": None}
        
        while True:
            # Process frames from queue (non-blocking)
            try:
                while True:
                    frame_data = frame_queue.get_nowait()
                    camera_id = frame_data["camera_id"]
                    current_frames[camera_id] = frame_data["frame"]
                    
            except Empty:
                pass  # No more frames in queue
            
            # Display frames only if display is enabled
            if ENABLE_DISPLAY:
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
            else:
                # In headless mode, just sleep to prevent busy waiting
                time.sleep(0.1)
                # You can still save frames programmatically in headless mode
                # display_manager.save_current_frames(current_frames)
    
    except KeyboardInterrupt:
        print("\nUser interrupted (Ctrl+C)")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("=" * 50)
        print("Shutting down, cleaning up resources...")
        
        # Stop background thread
        running_flag.clear()
        if streaming_thread and streaming_thread.is_alive():
            print("Waiting for streaming thread to stop...")
            streaming_thread.join(timeout=3.0)
        
        display_manager.cleanup()
        device_manager.cleanup()
        
        time.sleep(0.5)
        print("Cleanup complete.")
        print("=" * 50)


if __name__ == "__main__":
    main()