# Direct IPC Sender - Using Unix Domain Sockets for maximum performance
# Alternative to ZMQ for lower latency and overhead

import sys
import time
import struct
import numpy as np
import traceback
import signal
import threading
import socket
import os
import select

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
IPC_SOCKET_PATH = "/tmp/aria_slam_direct.sock"  # Direct IPC socket
STATS_PRINT_INTERVAL_SECONDS = 5.0

class DirectIPCSender:
    """Direct IPC sender using Unix Domain Sockets for minimum overhead."""
    
    def __init__(self, socket_path):
        self.socket_path = socket_path
        self.server_socket = None
        self.client_connections = []
        self.running = True
        
        # Statistics
        self.frames_sent_count = 0
        self.bytes_sent_count = 0
        self.start_time = time.time()
        
        # Thread for handling connections
        self.connection_thread = None
        self.lock = threading.Lock()
        
        # Add shutdown event for better coordination
        self._shutdown_event = threading.Event()
        
        self._setup_server()
        
    def _setup_server(self):
        """Setup Unix Domain Socket server."""
        # Clean up existing socket file
        if os.path.exists(self.socket_path):
            os.unlink(self.socket_path)
            print(f"Cleaned up existing socket: {self.socket_path}")
            
        # Create Unix Domain Socket
        self.server_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(self.socket_path)
        self.server_socket.listen(5)  # Allow up to 5 pending connections
        self.server_socket.settimeout(1.0)  # Non-blocking with timeout
        
        print(f"Direct IPC Server listening on: {self.socket_path}")
        
        # Start connection handling thread
        self.connection_thread = threading.Thread(target=self._handle_connections, daemon=True)
        self.connection_thread.start()
        
    def _handle_connections(self):
        """Handle incoming client connections."""
        while self.running and not self._shutdown_event.is_set():
            try:
                client_socket, _ = self.server_socket.accept()
                client_socket.settimeout(1.0)
                
                with self.lock:
                    self.client_connections.append(client_socket)
                    
                print(f"New client connected. Total clients: {len(self.client_connections)}")
                
            except socket.timeout:
                continue  # Check running flag and try again
            except Exception as e:
                if self.running and not self._shutdown_event.is_set():
                    print(f"Error accepting connection: {e}")
                break
                
    def send_frame(self, frame: np.ndarray, camera_id_str: str, timestamp: float):
        """Send frame data directly via Unix Domain Socket."""
        if not self.running or not self.client_connections or self._shutdown_event.is_set():
            return
            
        try:
            # Prepare frame data (same format as ZMQ for compatibility)
            frame_contiguous = np.ascontiguousarray(frame)
            height, width = frame_contiguous.shape[:2]
            channels = frame_contiguous.shape[2] if frame_contiguous.ndim == 3 else 1
            cam_id_byte = 1 if camera_id_str == 'left' else 2
            
            # Create header: timestamp(8) + cam_id(1) + height(2) + width(2) + channels(1) = 14 bytes
            header = struct.pack('!dBHHB', timestamp, cam_id_byte, height, width, channels)
            frame_bytes = frame_contiguous.tobytes()
            
            # Create message with length prefix for reliable streaming
            payload = header + frame_bytes
            message_length = len(payload)
            full_message = struct.pack('!I', message_length) + payload
            
            # Send to all connected clients
            with self.lock:
                disconnected_clients = []
                
                for client_socket in self.client_connections:
                    try:
                        # Use sendall for reliable delivery
                        client_socket.sendall(full_message)
                        
                    except (ConnectionResetError, BrokenPipeError, socket.timeout):
                        # Client disconnected
                        disconnected_clients.append(client_socket)
                    except Exception as e:
                        print(f"Error sending to client: {e}")
                        disconnected_clients.append(client_socket)
                
                # Remove disconnected clients
                for client in disconnected_clients:
                    try:
                        client.close()
                    except:
                        pass
                    if client in self.client_connections:
                        self.client_connections.remove(client)
                        
                if disconnected_clients:
                    print(f"Removed {len(disconnected_clients)} disconnected clients. "
                          f"Active clients: {len(self.client_connections)}")
            
            # Update statistics
            self.frames_sent_count += 1
            self.bytes_sent_count += len(full_message)
            
        except Exception as e:
            if self.running:
                print(f"Error in send_frame: {e}")
            
    def send_heartbeat(self):
        """Send heartbeat message."""
        if not self.running or not self.client_connections or self._shutdown_event.is_set():
            return
            
        try:
            # Heartbeat message: "HEARTBEAT" + timestamp
            heartbeat_data = struct.pack('!d', time.time())
            heartbeat_payload = b"HEARTBEAT" + heartbeat_data
            message_length = len(heartbeat_payload)
            full_message = struct.pack('!I', message_length) + heartbeat_payload
            
            with self.lock:
                disconnected_clients = []
                
                for client_socket in self.client_connections:
                    try:
                        client_socket.sendall(full_message)
                    except (ConnectionResetError, BrokenPipeError, socket.timeout):
                        disconnected_clients.append(client_socket)
                    except Exception as e:
                        if self.running:
                            print(f"Error sending heartbeat to client: {e}")
                        disconnected_clients.append(client_socket)
                
                # Remove disconnected clients
                for client in disconnected_clients:
                    try:
                        client.close()
                    except:
                        pass
                    if client in self.client_connections:
                        self.client_connections.remove(client)
                        
        except Exception as e:
            if self.running:
                print(f"Error sending heartbeat: {e}")
            
    def close(self):
        """Clean shutdown of Direct IPC sender."""
        print("Direct IPC Sender shutting down...")
        
        # Set shutdown flags
        self.running = False
        self._shutdown_event.set()
        
        # Wait briefly for ongoing operations to complete
        time.sleep(0.1)
        
        # Calculate statistics
        elapsed_time = time.time() - self.start_time
        avg_fps = self.frames_sent_count / elapsed_time if elapsed_time > 1 else self.frames_sent_count
        avg_mbps = (self.bytes_sent_count * 8 / (1024*1024)) / elapsed_time if elapsed_time > 1 else (self.bytes_sent_count * 8 / (1024*1024))
        
        print(f"  Total sent: {self.frames_sent_count} frames, {self.bytes_sent_count / (1024*1024):.2f} MB")
        print(f"  Runtime: {elapsed_time:.2f} seconds")
        print(f"  Average rate: {avg_fps:.1f} FPS, {avg_mbps:.2f} Mbps")
        
        # Close all client connections with timeout
        with self.lock:
            for client in self.client_connections:
                try:
                    client.settimeout(1.0)  # 1 second timeout
                    client.close()
                except:
                    pass
            self.client_connections.clear()
            
        # Close server socket with timeout
        if self.server_socket:
            try:
                self.server_socket.settimeout(1.0)
                self.server_socket.close()
                print("  Server socket closed.")
            except Exception as e:
                print(f"  Warning: Error closing server socket: {e}")
                
        # Clean up socket file
        try:
            if os.path.exists(self.socket_path):
                os.unlink(self.socket_path)
                print(f"  Cleaned up socket file: {self.socket_path}")
        except Exception as e:
            print(f"  Warning: Could not clean up socket file: {e}")
            
        # Wait for connection thread to finish with timeout
        if self.connection_thread and self.connection_thread.is_alive():
            self.connection_thread.join(timeout=2.0)
            if self.connection_thread.is_alive():
                print("  Warning: Connection thread did not terminate cleanly")
            
        print("Direct IPC Sender shutdown complete.")


class AriaStreamObserver:
    """Observer for receiving and caching latest images from Aria SDK."""
    
    def __init__(self):
        self.latest_data = {}
        self.frames_received_count_left = 0
        self.frames_received_count_right = 0

    def on_image_received(self, image: np.ndarray, record: ImageDataRecord):
        """Callback for new image data from Aria device."""
        camera_id_str = "left" if record.camera_id == aria.CameraId.Slam1 else "right"
        
        # Update frame counters
        if camera_id_str == "left":
            self.frames_received_count_left += 1
        else:
            self.frames_received_count_right += 1
            
        # Cache latest frame data
        self.latest_data[record.camera_id] = {
            "image": image,
            "timestamp": record.capture_timestamp_ns / 1e9,
            "sent": False
        }


# Global cleanup flag
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
    """Main execution function for Direct IPC sender."""
    
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    global _cleanup_in_progress
    
    if UPDATE_IPTABLES_ON_LINUX and sys.platform.startswith("linux"):
        update_iptables()
    
    # Set Aria SDK log level
    aria.set_log_level(aria.Level.Warning)
    
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

        # Initialize observer and Direct IPC sender
        observer = AriaStreamObserver()
        sender = DirectIPCSender(IPC_SOCKET_PATH)
        streaming_client.set_streaming_client_observer(observer)
        streaming_client.subscribe()
        print("Subscribed to SLAM data stream, starting Direct IPC transmission...")
        print("Clients can connect to:", IPC_SOCKET_PATH)

        # Statistics tracking variables
        last_stats_print_time = time.time()
        last_heartbeat_time = time.time()
        frames_sent_since_last_stats_left = 0
        frames_sent_since_last_stats_right = 0
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
                      f"Total sent: L={total_frames_sent_left} R={total_frames_sent_right} | "
                      f"Clients: {len(sender.client_connections)}")
                
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
        
        # Clean shutdown with timeout
        cleanup_start_time = time.time()
        max_cleanup_time = 5.0  # Maximum 5 seconds for cleanup
        
        if sender is not None:
            try:
                sender.close()
            except Exception as e:
                print(f"Error closing Direct IPC sender: {e}")
        
        # Clean shutdown of Aria device with timeout
        if device is not None:
            try:
                if streaming_manager:
                    print("  Stopping streaming...")
                    streaming_manager.stop_streaming()
                if recording_manager:
                    print("  Stopping recording...")
                    recording_manager.stop_recording()
                print("  Disconnecting device...")
                device_client.disconnect(device)
                print("Aria device disconnected.")
            except Exception as e:
                print(f"Error during device cleanup: {e}")
        else:
            print("Aria device was not connected.")
        
        # Check if cleanup is taking too long
        cleanup_elapsed = time.time() - cleanup_start_time
        if cleanup_elapsed > max_cleanup_time:
            print(f"Cleanup taking too long ({cleanup_elapsed:.1f}s), forcing exit...")
            import os
            os._exit(1)
        
        time.sleep(0.5)
        print("Cleanup complete.")
        print("=" * 50)
        
        # Force exit to ensure process terminates
        import os
        os._exit(0)

if __name__ == "__main__":
    main()
