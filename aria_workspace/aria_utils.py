"""
Aria Streaming Utilities

Shared utilities for Aria device streaming applications including:
- Device connection and configuration management
- Calibration loading and undistortion
- Common streaming observer base class
- Statistics tracking helpers
- Unified signal handling
- Display management for OpenCV windows
"""

import sys
import time
import json
import os
import numpy as np
import cv2
import aria.sdk as aria
from projectaria_tools.core.sensor_data import ImageDataRecord
from projectaria_tools.core.calibration import (
    device_calibration_from_json_string,
    distort_by_calibration,
    get_linear_camera_calibration,
)


class AriaDeviceManager:
    """Manages Aria device connection and streaming configuration."""
    
    def __init__(self, device_ip, streaming_profile, enable_rgb_stream=False):
        self.device_ip = device_ip
        self.streaming_profile = streaming_profile
        self.enable_rgb_stream = enable_rgb_stream
        self.device_client = aria.DeviceClient()
        self.device = None
        self.streaming_manager = None
        self.recording_manager = None
        self.streaming_client = None
        
    def connect(self):
        """Connect to Aria device and initialize managers."""
        print(f"Connecting to Aria device: {self.device_ip}...")
        client_config = aria.DeviceClientConfig()
        client_config.ip_v4_address = self.device_ip
        self.device_client.set_client_config(client_config)
        self.device = self.device_client.connect()
        print("Device connected successfully.")
        
        self.streaming_manager = self.device.streaming_manager
        self.recording_manager = self.device.recording_manager
        
    def stop_existing_sessions(self):
        """Stop any existing streaming or recording sessions."""
        print("Stopping any existing sessions...")
        
        try:
            self.streaming_manager.stop_streaming()
            print("  Existing streaming stopped.")
        except Exception as e:
            print(f"  Note (stopping stream): {e}")
            
        try:
            self.recording_manager.stop_recording()
            print("  Existing recording stopped.")
        except Exception as e:
            print(f"  Note (stopping recording): {e}")
        
        time.sleep(3)
        print("Pre-start cleanup finished.\n")
    
    def get_calibration(self, save_calibration=False, calibration_save_path=None):
        """Load device calibration data and optionally save to file."""
        device_calibration = None
        
        try:
            print("Loading device calibration...")
            sensors_calib_json = self.streaming_manager.sensors_calibration()
            
            # Save calibration data to file if enabled
            if save_calibration and sensors_calib_json and calibration_save_path:
                try:
                    # Parse and re-format JSON for better readability
                    calib_data = json.loads(sensors_calib_json)
                    
                    # Ensure directory exists
                    os.makedirs(os.path.dirname(calibration_save_path), exist_ok=True)
                    
                    # Save with pretty formatting
                    with open(calibration_save_path, 'w') as f:
                        json.dump(calib_data, f, indent=2, sort_keys=True)
                    
                    print(f"Device calibration saved to: {calibration_save_path}")
                    
                    # Also save a timestamped version
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    timestamped_path = calibration_save_path.replace('.json', f'_{timestamp}.json')
                    with open(timestamped_path, 'w') as f:
                        json.dump(calib_data, f, indent=2, sort_keys=True)
                    print(f"Timestamped calibration saved to: {timestamped_path}")
                    
                except Exception as e:
                    print(f"Warning: Failed to save calibration data: {e}")
            
            # Parse calibration for use
            device_calibration = device_calibration_from_json_string(sensors_calib_json)
            if device_calibration:
                print("Device calibration loaded successfully.")
            else:
                print("Warning: Could not parse device calibration.")
                
        except Exception as e:
            print(f"Warning: Failed to load calibration: {e}")
            
        return device_calibration
    
    def start_streaming(self):
        """Configure and start streaming with SLAM data subscription and optional RGB."""
        self.streaming_client = self.streaming_manager.streaming_client
        streaming_config = aria.StreamingConfig()
        streaming_config.profile_name = self.streaming_profile
        streaming_config.security_options.use_ephemeral_certs = True
        self.streaming_manager.streaming_config = streaming_config
        self.streaming_manager.start_streaming()
        print(f"Streaming started with profile: {self.streaming_profile}")

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
    
    def cleanup(self):
        """Clean shutdown of device connections."""
        if self.device is not None:
            try:
                if self.streaming_client is not None:
                    print("  Unsubscribing from Aria stream...")
                    self.streaming_client.unsubscribe()
                    print("    Successfully unsubscribed.")
            except Exception as e:
                print(f"    Error unsubscribing: {e}")
            
            try:
                if self.streaming_manager is not None:
                    print("  Stopping Aria device streaming...")
                    self.streaming_manager.stop_streaming()
                    print("    Streaming stopped successfully.")
            except Exception as e:
                print(f"    Error stopping stream: {e}")
            
            try:
                if self.recording_manager is not None:
                    self.recording_manager.stop_recording()
            except Exception as e:
                print(f"    Error stopping recording: {e}")
            
            try:
                print("  Disconnecting from Aria device...")
                self.device_client.disconnect(self.device)
                print("    Disconnected from device.")
            except Exception as e:
                print(f"    Error disconnecting: {e}")
        else:
            print("  Aria device was not connected.")


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

class CalibrationManager:
    """Manages camera calibration and undistortion operations."""
    
    def __init__(self, device_calibration, use_undistorted=False, 
                 left_output_width=640, left_output_height=480, left_focal_length=None,
                 right_output_width=640, right_output_height=480, right_focal_length=None,
                 rgb_output_width=512, rgb_output_height=512, rgb_focal_length=None):
        self.device_calibration = device_calibration
        self.use_undistorted = use_undistorted
        
        # Left camera configuration
        self.left_output_width = left_output_width
        self.left_output_height = left_output_height
        self.left_focal_length = left_focal_length
        
        # Right camera configuration  
        self.right_output_width = right_output_width
        self.right_output_height = right_output_height
        self.right_focal_length = right_focal_length
        
        # RGB camera configuration
        self.rgb_output_width = rgb_output_width
        self.rgb_output_height = rgb_output_height
        self.rgb_focal_length = rgb_focal_length
        
        # Calibration objects
        self.slam_left_calib = None
        self.slam_right_calib = None
        self.dst_left_calib = None
        self.dst_right_calib = None
        self.rgb_calib = None
        self.dst_rgb_calib = None
        
        if self.use_undistorted and self.device_calibration:
            self._initialize_calibrations()
    
    def _initialize_calibrations(self):
        """Initialize calibration objects for undistortion."""
        try:
            self.slam_left_calib = self.device_calibration.get_camera_calib("camera-slam-left")
            self.slam_right_calib = self.device_calibration.get_camera_calib("camera-slam-right")
            self.rgb_calib = self.device_calibration.get_camera_calib("camera-rgb")
            
            # Initialize left camera calibration
            if self.slam_left_calib:
                # Determine focal length to use for left camera
                if self.left_focal_length is None:
                    # Use original focal length from calibration
                    original_focal_lengths = self.slam_left_calib.get_focal_lengths()
                    focal_length = float(original_focal_lengths[0])
                else:
                    # Use provided focal length
                    focal_length = float(self.left_focal_length)
                
                # Create linear calibration for left undistorted output
                self.dst_left_calib = get_linear_camera_calibration(
                    self.left_output_width, self.left_output_height, focal_length, "camera-slam-left"
                )
                print(f"Left SLAM camera calibration loaded for undistortion "
                      f"(output: {self.left_output_width}x{self.left_output_height}, focal length: {focal_length:.1f})")
            
            # Initialize right camera calibration
            if self.slam_right_calib:
                # Determine focal length to use for right camera
                if self.right_focal_length is None:
                    # Use original focal length from calibration
                    original_focal_lengths = self.slam_right_calib.get_focal_lengths()
                    focal_length = float(original_focal_lengths[0])
                else:
                    # Use provided focal length
                    focal_length = float(self.right_focal_length)
                
                # Create linear calibration for right undistorted output
                self.dst_right_calib = get_linear_camera_calibration(
                    self.right_output_width, self.right_output_height, focal_length, "camera-slam-right"
                )
                print(f"Right SLAM camera calibration loaded for undistortion "
                      f"(output: {self.right_output_width}x{self.right_output_height}, focal length: {focal_length:.1f})")
            
            # Initialize RGB camera calibration
            if self.rgb_calib:
                # Determine focal length to use for RGB camera
                if self.rgb_focal_length is None:
                    # Use original focal length from calibration
                    original_focal_lengths = self.rgb_calib.get_focal_lengths()
                    focal_length = float(original_focal_lengths[0])
                else:
                    # Use provided focal length
                    focal_length = float(self.rgb_focal_length)
                
                self.dst_rgb_calib = get_linear_camera_calibration(
                    self.rgb_output_width, self.rgb_output_height, focal_length, "camera-rgb"
                )
                print(f"RGB camera calibration loaded for undistortion "
                      f"(output: {self.rgb_output_width}x{self.rgb_output_height}, focal length: {focal_length:.1f})")
                
        except Exception as e:
            print(f"Warning: Could not load camera calibrations for undistortion: {e}")
            self.use_undistorted = False
    
    def process_image(self, image: np.ndarray, camera_id_str: str) -> np.ndarray:
        """Apply undistortion to image if enabled, otherwise return original."""
        if not self.use_undistorted or not self.device_calibration:
            return image
            
        try:
            if camera_id_str == "left" and self.slam_left_calib and self.dst_left_calib:
                return distort_by_calibration(image, self.dst_left_calib, self.slam_left_calib)
            elif camera_id_str == "right" and self.slam_right_calib and self.dst_right_calib:
                return distort_by_calibration(image, self.dst_right_calib, self.slam_right_calib)
            elif camera_id_str == "rgb" and self.rgb_calib and self.dst_rgb_calib:
                return distort_by_calibration(image, self.dst_rgb_calib, self.rgb_calib)
        except Exception as e:
            print(f"Warning: Undistortion failed for {camera_id_str} camera: {e}")
            
        return image  # Fall back to raw image


class BaseAriaStreamObserver:
    """Base observer class for handling Aria stream data."""
    
    def __init__(self, calibration_manager):
        self.latest_data = {}
        self.frames_received_count_left = 0
        self.frames_received_count_right = 0
        self.frames_received_count_rgb = 0
        self.calibration_manager = calibration_manager
        
    def on_image_received(self, image: np.ndarray, record: ImageDataRecord):
        """Process incoming image data from Aria device."""
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
        
        # Store processed data
        self._store_frame_data(record, processed_image, camera_id_str)

    def _store_frame_data(self, record: ImageDataRecord, processed_image: np.ndarray, camera_id_str: str):
        """Store frame data - to be implemented by subclasses."""
        raise NotImplementedError("Subclasses must implement _store_frame_data")


class TransmissionStatistics:
    """Helper class for tracking and reporting transmission statistics."""
    
    def __init__(self, stats_interval=5.0):
        self.stats_interval = stats_interval
        self.last_stats_print_time = time.time()
        self.frames_sent_since_last_stats_left = 0
        self.frames_sent_since_last_stats_right = 0
        self.frames_sent_since_last_stats_rgb = 0
        self.total_frames_sent_left = 0
        self.total_frames_sent_right = 0
        self.total_frames_sent_rgb = 0
        self.last_received_count_left = 0
        self.last_received_count_right = 0
        self.last_received_count_rgb = 0
    
    def update_sent_count(self, camera_id_str: str):
        """Update sent frame counters."""
        if camera_id_str == "left":
            self.frames_sent_since_last_stats_left += 1
            self.total_frames_sent_left += 1
        elif camera_id_str == "right":
            self.frames_sent_since_last_stats_right += 1
            self.total_frames_sent_right += 1
        elif camera_id_str == "rgb":
            self.frames_sent_since_last_stats_rgb += 1
            self.total_frames_sent_rgb += 1
    
    def should_print_stats(self) -> bool:
        """Check if it's time to print statistics."""
        return time.time() - self.last_stats_print_time >= self.stats_interval
    
    def print_stats(self, observer):
        """Print transmission statistics."""
        current_time = time.time()
        time_diff = current_time - self.last_stats_print_time
        
        # Calculate rates
        received_left_fps = (observer.frames_received_count_left - self.last_received_count_left) / time_diff
        received_right_fps = (observer.frames_received_count_right - self.last_received_count_right) / time_diff
        received_rgb_fps = (observer.frames_received_count_rgb - self.last_received_count_rgb) / time_diff
        sent_left_fps = self.frames_sent_since_last_stats_left / time_diff
        sent_right_fps = self.frames_sent_since_last_stats_right / time_diff
        sent_rgb_fps = self.frames_sent_since_last_stats_rgb / time_diff
        
        timestamp = time.strftime("%H:%M:%S", time.localtime(current_time))
        print(f"STATUS [{timestamp}]: "
              f"L_Recv={received_left_fps:.1f}, L_Send={sent_left_fps:.1f} | "
              f"R_Recv={received_right_fps:.1f}, R_Send={sent_right_fps:.1f} | "
              f"RGB_Recv={received_rgb_fps:.1f}, RGB_Send={sent_rgb_fps:.1f} | "
              f"Total L={self.total_frames_sent_left}, R={self.total_frames_sent_right}, RGB={self.total_frames_sent_rgb}")
        
        # Reset counters
        self.frames_sent_since_last_stats_left = 0
        self.frames_sent_since_last_stats_right = 0
        self.frames_sent_since_last_stats_rgb = 0
        self.last_received_count_left = observer.frames_received_count_left
        self.last_received_count_right = observer.frames_received_count_right
        self.last_received_count_rgb = observer.frames_received_count_rgb
        self.last_stats_print_time = current_time


def setup_aria_sdk():
    """Initialize Aria SDK with appropriate log level."""
    aria.set_log_level(aria.Level.Warning)


def get_camera_id_string(camera_id) -> str:
    """Convert camera ID to string representation."""
    if camera_id == aria.CameraId.Slam1:
        return "left"
    elif camera_id == aria.CameraId.Slam2:
        return "right"
    elif camera_id == aria.CameraId.Rgb:
        return "rgb"
    else:
        return "unknown"


def get_camera_id_byte(camera_id_str: str) -> int:
    """Convert camera ID string to byte representation."""
    if camera_id_str == 'left':
        return 1
    elif camera_id_str == 'right':
        return 2
    elif camera_id_str == 'rgb':
        return 3
    else:
        return 0


class AriaDisplayManager:
    """Manages OpenCV display windows and frame visualization."""
    
    def __init__(self, display_scale=1.0, save_base_dir="/home/developer/aria_workspace", 
                 rotation_k=-1, enable_rgb_stream=True, enable_undistortion=True):
        self.display_scale = display_scale
        self.save_base_dir = save_base_dir
        self.rotation_k = rotation_k
        self.enable_rgb_stream = enable_rgb_stream
        self.undistortion_enabled = enable_undistortion
        
        # Window names
        self.stereo_window = "Aria SLAM Stereo Stream (Direct)"
        self.rgb_window = "Aria RGB Stream (Direct)"
        
        self._setup_windows()
        
    def _setup_windows(self):
        """Setup OpenCV display windows."""
        # Stereo display window
        cv2.namedWindow(self.stereo_window, cv2.WINDOW_NORMAL)
        stereo_width = int(960 * self.display_scale)
        stereo_height = int(640 * self.display_scale)
        cv2.resizeWindow(self.stereo_window, stereo_width, stereo_height)
        cv2.moveWindow(self.stereo_window, 50, 50)
        
        # RGB display window
        if self.enable_rgb_stream:
            cv2.namedWindow(self.rgb_window, cv2.WINDOW_NORMAL)
            rgb_width = int(512 * self.display_scale)
            rgb_height = int(512 * self.display_scale)
            cv2.resizeWindow(self.rgb_window, rgb_width, rgb_height)
            cv2.moveWindow(self.rgb_window, 1050, 50)
    
    def display_frames(self, frames):
        """Display current frames."""
        self._display_stereo(frames["left"], frames["right"])
        if self.enable_rgb_stream:
            self._display_rgb(frames["rgb"])
    
    def _display_stereo(self, left_frame, right_frame):
        """Display stereo pair (left and right cameras)."""
        if left_frame is not None and right_frame is not None:
            try:
                # Apply rotation for proper orientation
                left_rotated = np.rot90(left_frame, k=self.rotation_k)
                right_rotated = np.rot90(right_frame, k=self.rotation_k)
                
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
                rgb_rotated = np.rot90(rgb_frame, k=self.rotation_k)
                
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
            aria_folder = os.path.join(self.save_base_dir, "Aria_images")
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
                    rotated_frame = np.rot90(frame, k=self.rotation_k)
                    
                    # Handle RGB channel conversion for saving
                    if camera_id == "rgb":
                        # Convert RGB to BGR for correct color representation in saved image
                        if rotated_frame.ndim == 3 and rotated_frame.shape[2] == 3:
                            # Color RGB image - convert to BGR for OpenCV saving
                            save_frame = cv2.cvtColor(rotated_frame, cv2.COLOR_RGB2BGR)
                        else:
                            # Grayscale or single channel
                            save_frame = rotated_frame
                        filepath = os.path.join(folders["rgb"], f"RGB_{timestamp}.png")
                    else:
                        # SLAM cameras (left/right) - save as-is
                        save_frame = rotated_frame
                        if camera_id == "left":
                            filepath = os.path.join(folders["slam_left"], f"LEFT_{timestamp}.png")
                        else:  # right
                            filepath = os.path.join(folders["slam_right"], f"RIGHT_{timestamp}.png")
                    
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
