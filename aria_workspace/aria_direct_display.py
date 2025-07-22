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
    AriaDeviceManager, CalibrationManager, BaseAriaStreamObserver, 
    setup_aria_sdk
)

# ======================= Configuration =======================
STREAMING_PROFILE = "profile12"  # USB streaming profile
ROTATION_K = -1  # np.rot90 k parameter: -1=counterclockwise 90°, 1=clockwise 90°
SAVE_BASE_DIR = "/home/developer/aria_workspace"  # Base directory for saving images
ENABLE_RGB_STREAM = True  # Set to True to enable RGB camera stream
ENABLE_UNDISTORTION = True  # Set to True to apply undistortion by default
ENABLE_STATISTICS = True  # Set to True to show frame statistics
DISPLAY_SCALE = 1.0  # Scale factor for display windows (1.0 = original size)

# Undistortion configuration
LEFT_OUTPUT_WIDTH = 640
LEFT_OUTPUT_HEIGHT = 480
LEFT_FOCAL_LENGTH = None  # None = use original from calibration

RIGHT_OUTPUT_WIDTH = 640
RIGHT_OUTPUT_HEIGHT = 480
RIGHT_FOCAL_LENGTH = None

RGB_OUTPUT_WIDTH = 512
RGB_OUTPUT_HEIGHT = 512
RGB_FOCAL_LENGTH = 200
# =============================================================

class AriaUSBDeviceManager(AriaDeviceManager):
    """USB-specific device manager extending AriaDeviceManager."""
    
    def __init__(self, streaming_profile, enable_rgb_stream=False):
        # Initialize base class with dummy IP for USB connection
        super().__init__(None, streaming_profile, enable_rgb_stream)
        
    def connect(self):
        """Connect to Aria device via USB and initialize managers."""
        print("Connecting to Aria device via USB...")
        
        # Direct USB connection without IP configuration
        self.device = self.device_client.connect()
        print("Device connected successfully via USB.")
        
        # Print device information
        info = self.device.info
        status = self.device.status
        print(f"Device info - Model: {info.model}, Serial: {info.serial}")
        print(f"Device status - Battery: {status.battery_level}%, Mode: {status.device_mode}")
        
        self.streaming_manager = self.device.streaming_manager
        self.recording_manager = self.device.recording_manager
    
    def start_streaming(self):
        """Configure and start USB streaming with SLAM data subscription and optional RGB."""
        self.streaming_client = self.streaming_manager.streaming_client
        
        # Configure for USB streaming (key difference from base class)
        streaming_config = aria.StreamingConfig()
        streaming_config.profile_name = self.streaming_profile
        streaming_config.streaming_interface = aria.StreamingInterface.Usb
        streaming_config.security_options.use_ephemeral_certs = True
        self.streaming_manager.streaming_config = streaming_config
        
        try:
            self.streaming_manager.start_streaming()
            print(f"USB streaming started with profile: {self.streaming_profile}")
            
            # Wait and check state
            time.sleep(2)
            state = self.streaming_manager.streaming_state
            print(f"Current streaming state: {state}")
            
        except Exception as e:
            print(f"Error starting USB streaming: {e}")
            print("Note: If you get an active session error, please press the capture button on your Aria device to stop any active sessions, then restart this script.")
            raise

        # Configure streaming client for SLAM data subscription
        sub_config = self.streaming_client.subscription_config
        sub_config.subscriber_data_type = aria.StreamingDataType.Slam
        sub_config.message_queue_size[aria.StreamingDataType.Slam] = 5
        
        # Add RGB stream if enabled
        if self.enable_rgb_stream:
            sub_config.subscriber_data_type = sub_config.subscriber_data_type | aria.StreamingDataType.Rgb
            sub_config.message_queue_size[aria.StreamingDataType.Rgb] = 5
            print("RGB stream enabled")
        
        self.streaming_client.subscription_config = sub_config


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


class AriaDisplayObserver:
    """Legacy compatibility class - no longer used with threaded architecture."""
    pass


class AriaDisplayManager:
    """Manages OpenCV display windows and frame visualization."""
    
    def __init__(self):
        # Window names
        self.stereo_window = "Aria SLAM Stereo Stream (Direct)"
        self.rgb_window = "Aria RGB Stream (Direct)"
        
        self._setup_windows()
        self.undistortion_enabled = ENABLE_UNDISTORTION
        
    def _setup_windows(self):
        """Setup OpenCV display windows."""
        # Stereo display window
        cv2.namedWindow(self.stereo_window, cv2.WINDOW_NORMAL)
        stereo_width = int(960 * DISPLAY_SCALE)
        stereo_height = int(640 * DISPLAY_SCALE)
        cv2.resizeWindow(self.stereo_window, stereo_width, stereo_height)
        cv2.moveWindow(self.stereo_window, 50, 50)
        
        # RGB display window
        if ENABLE_RGB_STREAM:
            cv2.namedWindow(self.rgb_window, cv2.WINDOW_NORMAL)
            rgb_width = int(512 * DISPLAY_SCALE)
            rgb_height = int(512 * DISPLAY_SCALE)
            cv2.resizeWindow(self.rgb_window, rgb_width, rgb_height)
            cv2.moveWindow(self.rgb_window, 1050, 50)
    
    def display_frames(self, frames):
        """Display current frames."""
        self._display_stereo(frames["left"], frames["right"])
        if ENABLE_RGB_STREAM:
            self._display_rgb(frames["rgb"])
    
    def _display_stereo(self, left_frame, right_frame):
        """Display stereo pair (left and right cameras)."""
        if left_frame is not None and right_frame is not None:
            try:
                # Apply rotation for proper orientation
                left_rotated = np.rot90(left_frame, k=ROTATION_K)
                right_rotated = np.rot90(right_frame, k=ROTATION_K)
                
                # Concatenate frames horizontally
                stereo_display = np.hstack([left_rotated, right_rotated])
                
                # Ensure contiguous array for OpenCV
                if not stereo_display.flags['C_CONTIGUOUS']:
                    stereo_display = np.ascontiguousarray(stereo_display)
                
                # Add status text
                status_text = f"Undistortion: {'ON' if self.undistortion_enabled else 'OFF'}"
                cv2.putText(stereo_display, status_text, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, 255, 2)
                
                cv2.imshow(self.stereo_window, stereo_display)
                
            except Exception as e:
                print(f"Error displaying stereo frames: {e}")
        else:
            # Show waiting screen
            waiting_img = np.zeros((640, 960), dtype=np.uint8)
            cv2.putText(waiting_img, "Waiting for stereo frames...", (300, 320), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, 128, 2)
            cv2.imshow(self.stereo_window, waiting_img)
    
    def _display_rgb(self, rgb_frame):
        """Display RGB camera frame."""
        if rgb_frame is not None:
            try:
                # Apply rotation for proper orientation
                rgb_rotated = np.rot90(rgb_frame, k=ROTATION_K)
                
                # Handle different image formats
                if rgb_rotated.ndim == 2:
                    # Grayscale image
                    rgb_display = rgb_rotated
                elif rgb_rotated.ndim == 3 and rgb_rotated.shape[2] == 3:
                    # Color RGB image - convert to BGR for OpenCV
                    rgb_display = cv2.cvtColor(rgb_rotated, cv2.COLOR_RGB2BGR)
                else:
                    # Handle single channel as grayscale
                    rgb_display = rgb_rotated.squeeze()
                
                # Ensure contiguous array for OpenCV
                if not rgb_display.flags['C_CONTIGUOUS']:
                    rgb_display = np.ascontiguousarray(rgb_display)
                
                cv2.imshow(self.rgb_window, rgb_display)
                
            except Exception as e:
                print(f"Error displaying RGB frame: {e}")
        else:
            # Show waiting screen
            waiting_rgb = np.zeros((512, 512), dtype=np.uint8)
            cv2.putText(waiting_rgb, "Waiting for RGB...", (150, 256), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, 128, 2)
            cv2.imshow(self.rgb_window, waiting_rgb)
    
    def save_current_frames(self, frames):
        """Save current frames to disk."""
        try:
            timestamp = time.strftime("%Y%m%d_%H%M%S_%f")[:-3]
            
            # Create directory structure
            aria_folder = os.path.join(SAVE_BASE_DIR, "Aria_images")
            folders = {
                "slam_left": os.path.join(aria_folder, "slam_left"),
                "slam_right": os.path.join(aria_folder, "slam_right"),
                "rgb": os.path.join(aria_folder, "rgb")
            }
            
            for folder in folders.values():
                os.makedirs(folder, exist_ok=True)
            
            saved_files = []
            
            # Save frames with rotation applied
            for camera_id in ["left", "right", "rgb"]:
                frame = frames[camera_id]
                if frame is not None:
                    rotated_frame = np.rot90(frame, k=ROTATION_K)
                    
                    # Handle RGB channel conversion for saving
                    if camera_id == "rgb":
                        # Convert RGB to BGR for correct color representation in saved image
                        if rotated_frame.ndim == 3 and rotated_frame.shape[2] == 3:
                            # Color RGB image - convert to BGR for OpenCV saving
                            save_frame = cv2.cvtColor(rotated_frame, cv2.COLOR_RGB2BGR)
                        else:
                            # Grayscale or single channel
                            save_frame = rotated_frame
                        filepath = os.path.join(folders["rgb"], f"{timestamp}.png")
                    else:
                        # SLAM cameras (left/right) - save as-is
                        save_frame = rotated_frame
                        if camera_id == "left":
                            filepath = os.path.join(folders["slam_left"], f"{timestamp}.png")
                        else:  # right
                            filepath = os.path.join(folders["slam_right"], f"{timestamp}.png")
                    
                    cv2.imwrite(filepath, save_frame)
                    saved_files.append(filepath)
            
            if saved_files:
                print(f"Images saved: {len(saved_files)} files in {aria_folder}")
                return True
                
        except Exception as e:
            print(f"Error saving images: {e}")
        return False
    
    def toggle_undistortion(self):
        """Toggle undistortion on/off."""
        self.undistortion_enabled = not self.undistortion_enabled
        return self.undistortion_enabled
    
    def cleanup(self):
        """Cleanup display resources."""
        cv2.destroyAllWindows()


def main():
    """Main application entry point."""
    print("Aria Direct Display - Real-time camera stream visualization (Threaded)")
    print("Controls: 'q'=quit, 's'=save images, 'u'=toggle undistortion, ESC=quit")
    
    setup_aria_sdk()
    
    # Initialize managers
    device_manager = AriaUSBDeviceManager(STREAMING_PROFILE, ENABLE_RGB_STREAM)
    display_manager = AriaDisplayManager()
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