# Aria SLAM ZMQ Sender
#
# Connects to an Aria device, captures stereo camera images, and streams raw (uncompressed) image data
# via ZeroMQ (ZMQ) IPC to external receivers. Designed for use inside a Docker container.
# Sends periodic heartbeat messages for connection monitoring and prints transmission statistics.
#
# Prerequisites:
# - Aria SDK (projectaria_client_sdk)
# - pyzmq
#
# Configuration:
# - DEVICE_IP: IP address of the Aria device
# - STREAMING_PROFILE: Streaming profile name
# - UPDATE_IPTABLES_ON_LINUX: Whether to update iptables on Linux
# - ZMQ_BIND_ADDRESS: ZeroMQ bind address (IPC or TCP)
# - STATS_PRINT_INTERVAL_SECONDS: Interval for printing transmission statistics

import sys
import time
import struct
import numpy as np
import traceback
import cv2
import threading  # Add threading import for locks
try:
    import zmq
except ImportError:
    print("Error: pyzmq not installed. Install with: pip install pyzmq")
    sys.exit(1)
    
try:
    import aria.sdk as aria
    from projectaria_tools.core.sensor_data import ImageDataRecord
except ImportError:
    print("Error: Project Aria SDK not installed. Please install projectaria_client_sdk")
    sys.exit(1)
    
from common import update_iptables, quit_keypress, ctrl_c_handler
from aria_utils import (
    AriaDeviceManager, CalibrationManager, BaseAriaStreamObserver, 
    TransmissionStatistics, setup_aria_sdk, get_camera_id_string, get_camera_id_byte
)

# Configuration
DEVICE_IP = "192.168.3.41"
STREAMING_PROFILE = "profile14"
UPDATE_IPTABLES_ON_LINUX = True
ZMQ_BIND_ADDRESS = "ipc:///tmp/aria_slam.ipc"  # IPC socket for faster local communication
STATS_PRINT_INTERVAL_SECONDS = 5.0
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
RGB_UNDISTORT_OUTPUT_WIDTH = 640     # Output width for RGB undistorted images
RGB_UNDISTORT_OUTPUT_HEIGHT = 480    # Output height for RGB undistorted images
RGB_UNDISTORT_FOCAL_LENGTH = None    # Focal length for RGB undistorted output (None = use original from calibration)


class ZMQSender:
    """ZeroMQ sender for streaming SLAM image data with statistics tracking."""
    
    def __init__(self, bind_address):
        self.bind_address = bind_address
        self.context = zmq.Context()
        
        # Set critical ZMQ parameters to avoid hanging on exit
        self.context.setsockopt(zmq.LINGER, 1000)  # Force close after 1 second
        
        self.socket = self.context.socket(zmq.PUB)
        self.socket.set_hwm(10)  # High water mark to prevent memory buildup
        
        # Set socket timeouts to avoid infinite waiting
        self.socket.setsockopt(zmq.LINGER, 1000)  # Force close socket after 1 second
        self.socket.setsockopt(zmq.SNDHWM, 10)    # Send high water mark
        
        # Clean up any existing IPC socket file
        if bind_address.startswith("ipc://"):
            socket_path = bind_address[6:]  # Remove "ipc://" prefix
            try:
                import os
                if os.path.exists(socket_path):
                    os.unlink(socket_path)
                    print(f"Cleaned up existing socket: {socket_path}")
            except Exception as e:
                print(f"Warning: Could not clean up socket {socket_path}: {e}")
        
        self.socket.bind(bind_address)
        
        self.frames_sent_count = 0
        self.bytes_sent_count = 0
        self.start_time = time.time()
        
        print(f"ZMQ Sender initialized, bound to: {bind_address}")
        print("Waiting for subscribers to connect...")
        time.sleep(2)  # Give subscribers time to connect

    def send_frame(self, frame: np.ndarray, camera_id_str: str, timestamp: float):
        """Send a single frame with metadata via ZeroMQ."""
        try:
            # Ensure frame is contiguous in memory for efficient serialization
            frame_contiguous = np.ascontiguousarray(frame)
            height, width = frame_contiguous.shape[:2]
            channels = frame_contiguous.shape[2] if frame_contiguous.ndim == 3 else 1
            cam_id_byte = get_camera_id_byte(camera_id_str)
            
            # Pack frame metadata header
            header = struct.pack('!dBHHB', timestamp, cam_id_byte, height, width, channels)
            frame_bytes = frame_contiguous.tobytes()
            
            # Create complete message payload
            payload = header + frame_bytes
            
            # Send via ZeroMQ with topic-based routing
            topic = f"slam.{camera_id_str}"
            self.socket.send_multipart([topic.encode('utf-8'), payload], 
                                     zmq.NOBLOCK | zmq.DONTWAIT)
            
            # Update statistics
            self.frames_sent_count += 1
            self.bytes_sent_count += len(payload)
            
        except zmq.Again:
            # Non-blocking send failed (queue full), skip this frame
            print(f"Warning: ZMQ send queue full, dropping {camera_id_str} frame")
        except Exception as e:
            print(f"Error sending frame ({camera_id_str}): {e}")

    def send_heartbeat(self):
        """Send periodic heartbeat message for connection monitoring."""
        try:
            heartbeat_data = struct.pack('!d', time.time())
            self.socket.send_multipart([b"heartbeat", heartbeat_data], 
                                     zmq.NOBLOCK | zmq.DONTWAIT)
        except zmq.Again:
            pass  # Skip heartbeat if queue is full
        except Exception as e:
            print(f"Error sending heartbeat: {e}")

    def close(self):
        """Clean shutdown of ZMQ sender with statistics summary."""
        print("ZMQ Sender initiating shutdown...")
        
        # Wait briefly for ongoing operations to complete
        time.sleep(0.1)
        
        elapsed_time = time.time() - self.start_time
        avg_fps = self.frames_sent_count / elapsed_time if elapsed_time > 1 else self.frames_sent_count
        avg_mbps = (self.bytes_sent_count * 8 / (1024*1024)) / elapsed_time if elapsed_time > 1 else (self.bytes_sent_count * 8 / (1024*1024))
        
        print(f"  Total sent: {self.frames_sent_count} frames, {self.bytes_sent_count / (1024*1024):.2f} MB")
        print(f"  Runtime: {elapsed_time:.2f} seconds")
        print(f"  Average rate: {avg_fps:.1f} FPS, {avg_mbps:.2f} Mbps")
        
        try:
            # Attempt graceful socket closure
            if hasattr(self, 'socket') and self.socket:
                print("  Closing ZMQ socket...")
                self.socket.close(linger=1000)  # 1 second timeout
                print("  Socket closed.")
        except Exception as e:
            print(f"  Warning: Error closing socket: {e}")
        
        try:
            # Force terminate context with timeout
            if hasattr(self, 'context') and self.context:
                print("  Terminating ZMQ context...")
                self.context.term()
                print("  Context terminated.")
        except Exception as e:
            print(f"  Warning: Error terminating context: {e}")
        
        # Clean up IPC socket file
        if self.bind_address.startswith("ipc://"):
            socket_path = self.bind_address[6:]  # Remove "ipc://" prefix
            try:
                import os
                if os.path.exists(socket_path):
                    os.unlink(socket_path)
                    print(f"  Cleaned up IPC socket: {socket_path}")
            except Exception as e:
                print(f"  Warning: Could not clean up socket {socket_path}: {e}")
        
        print("ZMQ Sender shutdown complete.")


class AriaStreamObserver(BaseAriaStreamObserver):
    """Observer for receiving and caching latest images from Aria SDK with ZMQ-specific functionality."""
    
    def __init__(self, calibration_manager):
        super().__init__(calibration_manager)
        self.sent_frame_ids = {}  # Track which frames have been sent
        self.data_lock = threading.Lock()  # Add lock for thread-safe data access
    
    def _store_frame_data(self, record, processed_image, camera_id_str):
        """Store frame data for ZMQ transmission with unique frame tracking."""
        frame_id = f"{camera_id_str}_{record.capture_timestamp_ns}"
        with self.data_lock:  # Protect dictionary access
            self.latest_data[record.camera_id] = {
                "image": processed_image,
                "timestamp": record.capture_timestamp_ns / 1e9,
                "frame_id": frame_id,
                "sent": False
            }
    
    def get_latest_data_snapshot(self):
        """Get a thread-safe snapshot of latest data."""
        with self.data_lock:
            return dict(self.latest_data)  # Create a copy
    
    def mark_frame_sent(self, camera_id):
        """Thread-safely mark a frame as sent."""
        with self.data_lock:
            if camera_id in self.latest_data:
                self.latest_data[camera_id]["sent"] = True


def main():
    """Main execution function for Aria ZMQ sender."""
    
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
        sender = ZMQSender(ZMQ_BIND_ADDRESS)

        # Start streaming
        device_manager.start_streaming()
        device_manager.streaming_client.set_streaming_client_observer(observer)
        device_manager.streaming_client.subscribe()
        
        image_type_str = "undistorted" if USE_UNDISTORTED_IMAGES else "raw"
        stream_types = "SLAM"
        if ENABLE_RGB_STREAM:
            stream_types += " + RGB"
        print(f"Subscribed to {stream_types} data stream, starting ZMQ transmission ({image_type_str} images)...")

        # Initialize statistics tracking
        stats = TransmissionStatistics(STATS_PRINT_INTERVAL_SECONDS)
        last_heartbeat_time = time.time()
        
        # Use ctrl_c_handler from common module - unified signal handling
        with ctrl_c_handler() as ctrl_c:
            while not (quit_keypress() or ctrl_c):
                current_time = time.time()
                
                # Get thread-safe snapshot of data to avoid dictionary changed during iteration
                data_snapshot = observer.get_latest_data_snapshot()
                
                # Send available NEW frames only
                for camera_id, data in data_snapshot.items():
                    if not data.get("sent", False):  # Only send if not already sent
                        # Use the corrected camera ID mapping
                        if camera_id == aria.CameraId.Slam1:
                            camera_id_str = "left"
                        elif camera_id == aria.CameraId.Slam2:
                            camera_id_str = "right"
                        elif camera_id == aria.CameraId.Rgb:
                            camera_id_str = "rgb"
                        else:
                            continue  # Skip unknown camera types
                        
                        sender.send_frame(data["image"], camera_id_str, data["timestamp"])
                        
                        # Mark frame as sent using thread-safe method
                        observer.mark_frame_sent(camera_id)
                        stats.update_sent_count(camera_id_str)
                
                # Send periodic heartbeat
                if current_time - last_heartbeat_time > 1.0:
                    sender.send_heartbeat()
                    last_heartbeat_time = current_time
                
                # Print statistics periodically
                if stats.should_print_stats():
                    stats.print_stats(observer)
                
                time.sleep(0.01)  # Small delay to prevent excessive CPU usage

    except KeyboardInterrupt:
        print("\nUser interrupted (Ctrl+C).")
    except Exception as e:
        print(f"Error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("=" * 50)
        print("Shutting down, cleaning up resources...")
        
        # Clean shutdown of ZMQ sender
        if sender is not None:
            try:
                sender.close()
            except Exception as e:
                print(f"Error closing ZMQ sender: {e}")
        else:
            print("ZMQ sender was not initialized.")

        device_manager.cleanup()
        
        time.sleep(0.5)
        print("Cleanup complete.")
        print("=" * 50)

if __name__ == "__main__":
    main()
