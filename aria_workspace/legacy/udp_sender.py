# Aria SLAM UDP Sender
#
# Connects to an Aria device, captures stereo camera images, compresses them using JPEG,
# and streams the compressed data over UDP to a specified target.
# Tracks and prints transmission statistics. Supports Linux iptables update for local forwarding.
#

import sys
import time
import socket
import struct
import numpy as np
import cv2  # Added for JPEG compression
import aria.sdk as aria
from projectaria_tools.core.sensor_data import ImageDataRecord
from common import update_iptables, quit_keypress, ctrl_c_handler
from aria_utils import (
    AriaDeviceManager, CalibrationManager, BaseAriaStreamObserver, 
    TransmissionStatistics, setup_aria_sdk, get_camera_id_string, get_camera_id_byte
)

# Configuration
DEVICE_IP = "192.168.3.41"
STREAMING_PROFILE = "profile18"
UPDATE_IPTABLES_ON_LINUX = True
FORWARDING_IP = "127.0.0.1"
FORWARDING_PORT = 9999
STATS_PRINT_INTERVAL_SECONDS = 5.0
JPEG_QUALITY = 85  # JPEG compression quality (1-100, higher = better quality but larger size)
USE_UNDISTORTED_IMAGES = True  # Set to True to send undistorted images, False for raw images
SAVE_CALIBRATION = False  # Set to True to save calibration data to file
CALIBRATION_SAVE_PATH = "/home/developer/aria_workspace/device_calibration.json"  # Path to save calibration
ENABLE_RGB_STREAM = True  # Set to True to enable RGB stream capture and transmission

# Left SLAM camera undistortion configuration - used when USE_UNDISTORTED_IMAGES is True
LEFT_UNDISTORT_OUTPUT_WIDTH = 640    # Output width for left undistorted images
LEFT_UNDISTORT_OUTPUT_HEIGHT = 480   # Output height for left undistorted images  
LEFT_UNDISTORT_FOCAL_LENGTH = None   # Focal length for left undistorted output (None = use original from calibration)

# Right SLAM camera undistortion configuration - used when USE_UNDISTORTED_IMAGES is True
RIGHT_UNDISTORT_OUTPUT_WIDTH = 640    # Output width for right undistorted images
RIGHT_UNDISTORT_OUTPUT_HEIGHT = 480   # Output height for right undistorted images
RIGHT_UNDISTORT_FOCAL_LENGTH = None   # Focal length for right undistorted output (None = use original from calibration)

# RGB camera undistortion configuration - used when USE_UNDISTORTED_IMAGES is True and ENABLE_RGB_STREAM is True
RGB_UNDISTORT_OUTPUT_WIDTH = 512     # Output width for RGB undistorted images
RGB_UNDISTORT_OUTPUT_HEIGHT = 512    # Output height for RGB undistorted images
RGB_UNDISTORT_FOCAL_LENGTH = None    # Focal length for RGB undistorted output (None = use original from calibration)


class UDPSender:
    """UDP sender for compressed image data with statistics tracking."""
    def __init__(self, host, port):
        self.target_address = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.MAX_UDP_PAYLOAD = 65507 - 28
        
        self.frames_sent_count = 0
        self.bytes_sent_count = 0
        self.compression_ratio_sum = 0.0
        self.start_time = time.time()
        
        print(f"UDP Sender initialized with JPEG compression (quality={JPEG_QUALITY}), target: {host}:{port}")

    def send_frame(self, frame: np.ndarray, camera_id_str: str, timestamp: float):
        try:
            # Convert frame to uint8 if needed
            if frame.dtype != np.uint8:
                frame = frame.astype(np.uint8)
            
            # Get original frame info
            original_height, original_width = frame.shape[:2]
            original_channels = frame.shape[2] if frame.ndim == 3 else 1
            original_size = frame.nbytes
            
            # Convert grayscale to BGR for JPEG encoding if needed
            if frame.ndim == 2:
                frame_for_encoding = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            else:
                frame_for_encoding = frame
            
            # JPEG compress the image
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
            success, compressed_data = cv2.imencode('.jpg', frame_for_encoding, encode_param)
            
            if not success:
                print(f"Failed to compress frame ({camera_id_str})")
                return
            
            compressed_bytes = compressed_data.tobytes()
            compressed_size = len(compressed_bytes)
            
            # Calculate compression ratio
            compression_ratio = original_size / compressed_size if compressed_size > 0 else 1.0
            self.compression_ratio_sum += compression_ratio
            
            cam_id_byte = get_camera_id_byte(camera_id_str)
            
            # Pack header: timestamp, cam_id, original_height, original_width, original_channels, compressed_size
            header = struct.pack('!dBHHBI', timestamp, cam_id_byte, original_height, original_width, original_channels, compressed_size)
            payload = header + compressed_bytes

            # Check if payload fits in single UDP packet
            if len(payload) > self.MAX_UDP_PAYLOAD:
                print(f"Warning: Compressed frame still too large ({len(payload)} bytes), consider reducing JPEG_QUALITY")
                return
            
            # Send as single packet
            self.sock.sendto(b'COMP' + payload, self.target_address)
            
            self.frames_sent_count += 1
            self.bytes_sent_count += len(payload)

        except Exception as e:
            print(f"Error sending compressed frame ({camera_id_str}): {e}")

    def close(self):
        self.sock.close()
        elapsed_time = time.time() - self.start_time
        avg_fps = self.frames_sent_count / elapsed_time if elapsed_time > 1 else self.frames_sent_count
        avg_mbps = (self.bytes_sent_count * 8 / (1024*1024)) / elapsed_time if elapsed_time > 1 else (self.bytes_sent_count * 8 / (1024*1024))
        avg_compression_ratio = self.compression_ratio_sum / self.frames_sent_count if self.frames_sent_count > 0 else 1.0
        
        print(f"UDP Sender closed.")
        print(f"  Total sent: {self.frames_sent_count} frames, {self.bytes_sent_count / (1024*1024):.2f} MB")
        print(f"  Runtime: {elapsed_time:.2f} seconds")
        print(f"  Average rate: {avg_fps:.1f} FPS, {avg_mbps:.2f} Mbps")
        print(f"  Average compression ratio: {avg_compression_ratio:.1f}x")


class AriaStreamObserver(BaseAriaStreamObserver):
    """Observer for caching latest images from Aria SDK with UDP-specific functionality."""
    
    def _store_frame_data(self, record, processed_image, camera_id_str):
        """Store frame data for UDP transmission."""
        self.latest_data[record.camera_id] = {
            "image": processed_image,
            "timestamp": record.capture_timestamp_ns / 1e9
        }


def main():
    if UPDATE_IPTABLES_ON_LINUX and sys.platform.startswith("linux"):
        update_iptables()
    
    setup_aria_sdk()
    
    # Initialize managers
    device_manager = AriaDeviceManager(DEVICE_IP, STREAMING_PROFILE, ENABLE_RGB_STREAM)
    sender = None
    observer = None
    calibration_manager = None

    try:
        # Connect and setup device
        device_manager.connect()
        device_manager.stop_existing_sessions()

        # Handle calibration
        device_calibration = None
        if USE_UNDISTORTED_IMAGES or SAVE_CALIBRATION:
            device_calibration = device_manager.get_calibration(SAVE_CALIBRATION, CALIBRATION_SAVE_PATH)
            if USE_UNDISTORTED_IMAGES and not device_calibration:
                print("Using raw images due to calibration failure.")

        # Initialize calibration manager and observer
        calibration_manager = CalibrationManager(
            device_calibration, 
            USE_UNDISTORTED_IMAGES,
            LEFT_UNDISTORT_OUTPUT_WIDTH, LEFT_UNDISTORT_OUTPUT_HEIGHT, LEFT_UNDISTORT_FOCAL_LENGTH,
            RIGHT_UNDISTORT_OUTPUT_WIDTH, RIGHT_UNDISTORT_OUTPUT_HEIGHT, RIGHT_UNDISTORT_FOCAL_LENGTH,
            RGB_UNDISTORT_OUTPUT_WIDTH, RGB_UNDISTORT_OUTPUT_HEIGHT, RGB_UNDISTORT_FOCAL_LENGTH
        )
        observer = AriaStreamObserver(calibration_manager)
        sender = UDPSender(FORWARDING_IP, FORWARDING_PORT)

        # Start streaming
        device_manager.start_streaming()
        device_manager.streaming_client.set_streaming_client_observer(observer)
        device_manager.streaming_client.subscribe()
        
        image_type_str = "undistorted" if USE_UNDISTORTED_IMAGES else "raw"
        stream_types = "SLAM"
        if ENABLE_RGB_STREAM:
            stream_types += " + RGB"
        print(f"Subscribed to {stream_types} data stream, starting UDP transmission ({image_type_str} images with JPEG compression)...")

        # Initialize statistics tracking
        stats = TransmissionStatistics(STATS_PRINT_INTERVAL_SECONDS)
        
        # Use unified signal handling from common module
        with ctrl_c_handler() as ctrl_c:
            while not (quit_keypress() or ctrl_c):
                current_loop_time = time.time()
                available_cameras = list(observer.latest_data.keys())
                
                processed_in_loop = False
                for cam_id in available_cameras:
                    data = observer.latest_data.pop(cam_id, None)
                    if data:
                        processed_in_loop = True
                        # Use the corrected get_camera_id_string function
                        if cam_id == aria.CameraId.Slam1:
                            camera_id_str = "left"
                        elif cam_id == aria.CameraId.Slam2:
                            camera_id_str = "right"
                        elif cam_id == aria.CameraId.Rgb:
                            camera_id_str = "rgb"
                        else:
                            continue  # Skip unknown camera types
                        
                        sender.send_frame(data["image"], camera_id_str, data["timestamp"])
                        stats.update_sent_count(camera_id_str)

                if stats.should_print_stats():
                    stats.print_stats(observer)

                if not processed_in_loop:
                    time.sleep(0.005)
                else:
                    time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nUser interrupted (Ctrl+C).")
    except Exception as e:
        print(f"Error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("=" * 50)
        print("Shutting down, cleaning up resources...")
        
        if sender is not None:
            print("  Closing UDP sender...")
            sender.close()
        else:
            print("  UDP sender was not initialized.")

        device_manager.cleanup()
        
        time.sleep(0.5)
        print("Cleanup complete.")
        print("=" * 50)

if __name__ == "__main__":
    main()