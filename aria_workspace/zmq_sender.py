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
import signal
import threading
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
    
from common import update_iptables, quit_keypress

# Configuration
DEVICE_IP = "192.168.3.41"
STREAMING_PROFILE = "profile14"
UPDATE_IPTABLES_ON_LINUX = True
ZMQ_BIND_ADDRESS = "ipc:///tmp/aria_slam.ipc"  # IPC socket for faster local communication
STATS_PRINT_INTERVAL_SECONDS = 5.0

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
        
        # Add shutdown flag
        self._shutdown_event = threading.Event()
        
        print(f"ZMQ Sender initialized, bound to: {bind_address}")
        print("Waiting for subscribers to connect...")
        time.sleep(2)  # Give subscribers time to connect

    def send_frame(self, frame: np.ndarray, camera_id_str: str, timestamp: float):
        """Send a single frame with metadata via ZeroMQ."""
        # Check if shutting down
        if self._shutdown_event.is_set():
            return
            
        try:
            # Ensure frame is contiguous in memory for efficient serialization
            frame_contiguous = np.ascontiguousarray(frame)
            height, width = frame_contiguous.shape[:2]
            channels = frame_contiguous.shape[2] if frame_contiguous.ndim == 3 else 1
            cam_id_byte = 1 if camera_id_str == 'left' else 2
            
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
            if not self._shutdown_event.is_set():
                print(f"Warning: ZMQ send queue full, dropping {camera_id_str} frame")
        except Exception as e:
            if not self._shutdown_event.is_set():
                print(f"Error sending frame ({camera_id_str}): {e}")

    def send_heartbeat(self):
        """Send periodic heartbeat message for connection monitoring."""
        if self._shutdown_event.is_set():
            return
            
        try:
            heartbeat_data = struct.pack('!d', time.time())
            self.socket.send_multipart([b"heartbeat", heartbeat_data], 
                                     zmq.NOBLOCK | zmq.DONTWAIT)
        except zmq.Again:
            pass  # Skip heartbeat if queue is full
        except Exception as e:
            if not self._shutdown_event.is_set():
                print(f"Error sending heartbeat: {e}")

    def close(self):
        """Clean shutdown of ZMQ sender with statistics summary."""
        print("ZMQ Sender initiating shutdown...")
        
        # Set shutdown flag
        self._shutdown_event.set()
        
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
                # Terminate context in separate thread to avoid infinite waiting
                def terminate_context():
                    try:
                        self.context.term()
                        print("  Context terminated.")
                    except Exception as e:
                        print(f"  Warning: Error terminating context: {e}")
                
                terminate_thread = threading.Thread(target=terminate_context)
                terminate_thread.daemon = True
                terminate_thread.start()
                terminate_thread.join(timeout=2.0)  # Wait maximum 2 seconds
                
                if terminate_thread.is_alive():
                    print("  Warning: Context termination timed out, forcing exit")
                    
        except Exception as e:
            print(f"  Warning: Error during context cleanup: {e}")
        
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


class AriaStreamObserver:
    """Observer for receiving and caching latest images from Aria SDK."""
    
    def __init__(self):
        self.latest_data = {}
        self.frames_received_count_left = 0
        self.frames_received_count_right = 0
        self.sent_frame_ids = {}  # Track which frames have been sent

    def on_image_received(self, image: np.ndarray, record: ImageDataRecord):
        """Callback for new image data from Aria device."""
        camera_id_str = "left" if record.camera_id == aria.CameraId.Slam1 else "right"
        
        # Update frame counters
        if camera_id_str == "left":
            self.frames_received_count_left += 1
        else:
            self.frames_received_count_right += 1
            
        # Cache latest frame data with unique frame ID
        frame_id = f"{camera_id_str}_{record.capture_timestamp_ns}"
        self.latest_data[record.camera_id] = {
            "image": image,
            "timestamp": record.capture_timestamp_ns / 1e9,
            "frame_id": frame_id,
            "sent": False
        }


# Add global variable to track cleanup status
_cleanup_in_progress = False

def signal_handler(signum, frame):
    """Handle signals to ensure proper exit"""
    global _cleanup_in_progress
    if _cleanup_in_progress:
        print("\nForced exit...")
        import os
        os._exit(1)
    else:
        print(f"\nReceived signal {signum}, initiating cleanup...")
        _cleanup_in_progress = True

def main():
    """Main execution function for Aria ZMQ sender."""
    
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    global _cleanup_in_progress
    
    if UPDATE_IPTABLES_ON_LINUX and sys.platform.startswith("linux"):
        update_iptables()
    
    # Set Aria SDK log level
    aria.set_log_level(aria.Level.Warning)
    
    # Initialize components
    device_client = aria.DeviceClient()
    sender = None
    device = None
    streaming_manager = None
    recording_manager = None
    streaming_client = None

    try:
        print(f"Connecting to Aria device: {DEVICE_IP}...")
        client_config = aria.DeviceClientConfig()
        client_config.ip_v4_address = DEVICE_IP
        device_client.set_client_config(client_config)
        device = device_client.connect()
        print("Device connected successfully.")

        print("Stopping any existing sessions...")
        streaming_manager = device.streaming_manager
        recording_manager = device.recording_manager
        
        # Clean stop of existing streaming/recording
        try:
            streaming_manager.stop_streaming()
            print("  Existing streaming stopped.")
        except Exception as e:
            print(f"  Note (stopping stream): {e}")
            
        try:
            recording_manager.stop_recording()
            print("  Existing recording stopped.")
        except Exception as e:
            print(f"  Note (stopping recording): {e}")
        
        time.sleep(3)
        print("Pre-start cleanup finished.\n")

        # Configure and start streaming
        streaming_client = streaming_manager.streaming_client
        streaming_config = aria.StreamingConfig()
        streaming_config.profile_name = STREAMING_PROFILE
        streaming_config.security_options.use_ephemeral_certs = True
        streaming_manager.streaming_config = streaming_config
        streaming_manager.start_streaming()
        print(f"Streaming started with profile: {STREAMING_PROFILE}")

        # Configure streaming client for SLAM data subscription
        sub_config = streaming_client.subscription_config
        sub_config.subscriber_data_type = aria.StreamingDataType.Slam
        sub_config.message_queue_size[aria.StreamingDataType.Slam] = 5
        streaming_client.subscription_config = sub_config

        # Initialize observer and ZMQ sender
        observer = AriaStreamObserver()
        sender = ZMQSender(ZMQ_BIND_ADDRESS)
        streaming_client.set_streaming_client_observer(observer)
        streaming_client.subscribe()
        print("Subscribed to SLAM data stream, starting ZMQ transmission...")

        # Statistics tracking variables
        last_stats_print_time = time.time()
        last_heartbeat_time = time.time()
        frames_sent_since_last_stats_left = 0
        frames_sent_since_last_stats_right = 0
        frames_received_since_last_stats_left = 0
        frames_received_since_last_stats_right = 0
        total_frames_sent_left = 0
        total_frames_sent_right = 0
        last_received_count_left = 0
        last_received_count_right = 0
        
        # Main transmission loop
        while not quit_keypress() and not _cleanup_in_progress:
            current_time = time.time()
            
            # Check if shutting down
            if _cleanup_in_progress:
                break
            
            # Send available NEW frames only
            for camera_id, data in observer.latest_data.items():
                if not data.get("sent", False):  # Only send if not already sent
                    camera_id_str = "left" if camera_id == aria.CameraId.Slam1 else "right"
                    sender.send_frame(data["image"], camera_id_str, data["timestamp"])
                    
                    # Mark frame as sent
                    data["sent"] = True
                    
                    if camera_id_str == "left":
                        frames_sent_since_last_stats_left += 1
                        total_frames_sent_left += 1
                    else:
                        frames_sent_since_last_stats_right += 1
                        total_frames_sent_right += 1
            
            # Send periodic heartbeat
            if current_time - last_heartbeat_time > 1.0:
                sender.send_heartbeat()
                last_heartbeat_time = current_time
            
            # Print statistics periodically
            if current_time - last_stats_print_time > STATS_PRINT_INTERVAL_SECONDS:
                time_diff = current_time - last_stats_print_time
                
                # Calculate rates
                received_left_fps = (observer.frames_received_count_left - last_received_count_left) / time_diff
                received_right_fps = (observer.frames_received_count_right - last_received_count_right) / time_diff
                sent_left_fps = frames_sent_since_last_stats_left / time_diff
                sent_right_fps = frames_sent_since_last_stats_right / time_diff
                
                timestamp = time.strftime("%H:%M:%S", time.localtime(current_time))
                print(f"[{timestamp}] Received: L={received_left_fps:.1f} R={received_right_fps:.1f} FPS | "
                      f"Sent: L={sent_left_fps:.1f} R={sent_right_fps:.1f} FPS | "
                      f"Total sent: L={total_frames_sent_left} R={total_frames_sent_right}")
                
                # Reset counters
                frames_sent_since_last_stats_left = 0
                frames_sent_since_last_stats_right = 0
                last_received_count_left = observer.frames_received_count_left
                last_received_count_right = observer.frames_received_count_right
                last_stats_print_time = current_time
            
            time.sleep(0.01)  # Small delay to prevent excessive CPU usage

    except KeyboardInterrupt:
        print("\nUser interrupted (Ctrl+C).")
        _cleanup_in_progress = True
    except Exception as e:
        print(f"Error occurred: {e}")
        traceback.print_exc()
        _cleanup_in_progress = True
    finally:
        _cleanup_in_progress = True
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

        # Clean shutdown of Aria device
        if device is not None:
            try:
                if streaming_manager:
                    streaming_manager.stop_streaming()
                if recording_manager:
                    recording_manager.stop_recording()
                device_client.disconnect(device)
                print("Aria device disconnected.")
            except Exception as e:
                print(f"Error during device cleanup: {e}")
        else:
            print("Aria device was not connected.")
        
        time.sleep(0.5)
        print("Cleanup complete.")
        print("=" * 50)
        
        # Ensure process can exit
        import os
        os._exit(0)

if __name__ == "__main__":
    main()
