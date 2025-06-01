# Direct IPC Receiver - Receives SLAM data via Unix Domain Socket
# Companion receiver for direct_ipc_sender.py

import sys
import time
import struct
import numpy as np
import socket
import os
import signal
import threading
from collections import deque

try:
    import cv2
except ImportError:
    print("Error: OpenCV not installed. Install with: pip install opencv-python")
    sys.exit(1)

# Configuration
IPC_SOCKET_PATH = "/tmp/aria_slam_direct.sock"
STATS_PRINT_INTERVAL_SECONDS = 5.0
DISPLAY_ENABLED = True
SAVE_FRAMES = False
OUTPUT_DIR = "/tmp/aria_frames"
ROTATION_K = -1  # np.rot90 k parameter: -1=counterclockwise 90°, 1=clockwise 90°
VERBOSE_LOGGING = False  # Set to True for detailed frame parsing logs

class DirectIPCReceiver:
    """Direct IPC receiver using Unix Domain Socket for maximum performance."""
    
    def __init__(self, socket_path):
        self.socket_path = socket_path
        self.client_socket = None
        self.running = True
        self.connected = False
        
        # Statistics
        self.frames_received_count = 0
        self.bytes_received_count = 0
        self.start_time = time.time()
        self.last_frame_time = {}
        
        # Frame data
        self.latest_frames = {}
        self.frame_buffer = deque(maxlen=100)
        
        # Threading
        self.receive_thread = None
        self.display_thread = None
        self.lock = threading.Lock()
        
        # Frame counters for FPS calculation
        self.frame_count = {"left": 0, "right": 0}
        self.last_frame_count = {"left": 0, "right": 0}
        self.last_stats_time = time.time()
        
        # Current frames for display
        self.current_left_frame = None
        self.current_right_frame = None
        
        # Setup output directory if saving frames
        if SAVE_FRAMES:
            os.makedirs(OUTPUT_DIR, exist_ok=True)
            
        print(f"Direct IPC Receiver initialized for: {socket_path}")
        
    def connect(self):
        """Connect to the IPC server."""
        max_retries = 10
        retry_delay = 1.0
        
        for attempt in range(max_retries):
            try:
                print(f"Attempting to connect to {self.socket_path} (attempt {attempt + 1}/{max_retries})")
                
                self.client_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                self.client_socket.connect(self.socket_path)
                self.client_socket.settimeout(5.0)  # 5 second timeout for receives
                
                self.connected = True
                print("Successfully connected to IPC server")
                
                # Start receiving thread
                self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
                self.receive_thread.start()
                
                # Start display thread if enabled
                if DISPLAY_ENABLED:
                    self.display_thread = threading.Thread(target=self._display_loop, daemon=True)
                    self.display_thread.start()
                
                return True
                
            except (ConnectionRefusedError, FileNotFoundError):
                print(f"  Connection failed, retrying in {retry_delay} seconds...")
                time.sleep(retry_delay)
                retry_delay = min(retry_delay * 1.5, 5.0)  # Exponential backoff
                
            except Exception as e:
                print(f"  Error connecting: {e}")
                time.sleep(retry_delay)
                
        print("Failed to connect after all retry attempts")
        return False
        
    def _receive_loop(self):
        """Main receive loop running in separate thread."""
        buffer = b""
        
        while self.running and self.connected:
            try:
                # Receive data
                data = self.client_socket.recv(65536)  # 64KB buffer
                if not data:
                    print("Server disconnected")
                    self.connected = False
                    break
                    
                buffer += data
                
                # Process complete messages from buffer
                while len(buffer) >= 4:  # At least message length header
                    # Read message length
                    message_length = struct.unpack('!I', buffer[:4])[0]
                    
                    # Check if we have complete message
                    if len(buffer) < 4 + message_length:
                        break  # Need more data
                        
                    # Extract complete message
                    message_data = buffer[4:4 + message_length]
                    buffer = buffer[4 + message_length:]
                    
                    # Process the message
                    self._process_message(message_data)
                    
            except socket.timeout:
                continue  # Try again
            except Exception as e:
                if self.running:
                    print(f"Error in receive loop: {e}")
                self.connected = False
                break
                
        print("Receive loop ended")
        
    def _process_message(self, message_data):
        """Process a complete message."""
        try:
            # Check if it's a heartbeat
            if message_data.startswith(b"HEARTBEAT"):
                # Heartbeat format: b"HEARTBEAT" + struct.pack('!d', time.time())
                if len(message_data) >= 9 + 8:  # "HEARTBEAT" (9 bytes) + double (8 bytes)
                    heartbeat_time = struct.unpack('!d', message_data[9:17])[0]
                    if VERBOSE_LOGGING:
                        print(f"Heartbeat received: {heartbeat_time}")
                else:
                    print(f"Invalid heartbeat message length: {len(message_data)}")
                return
                
            # Parse frame header
            if len(message_data) < 14:  # Header size: 8+1+2+2+1 = 14 bytes
                print(f"Message too short for frame header: {len(message_data)} bytes, expected at least 14")
                return
                
            # Correct header size is 14 bytes: timestamp(8) + cam_id(1) + height(2) + width(2) + channels(1)
            header = message_data[:14]
            frame_data = message_data[14:]
            
            # Unpack header: timestamp, cam_id, height, width, channels
            timestamp, cam_id_byte, height, width, channels = struct.unpack('!dBHHB', header)
            
            camera_id_str = "left" if cam_id_byte == 1 else "right"
            
            # Reconstruct frame
            expected_size = height * width * channels
            if len(frame_data) != expected_size:
                print(f"Frame data size mismatch: expected {expected_size}, got {len(frame_data)} for {camera_id_str} {height}x{width}x{channels}")
                return
                
            # Convert bytes back to numpy array
            frame_shape = (height, width, channels) if channels > 1 else (height, width)
            frame = np.frombuffer(frame_data, dtype=np.uint8).reshape(frame_shape)
            
            # Only log detailed info if verbose logging is enabled
            if VERBOSE_LOGGING:
                print(f"Frame parsed: {camera_id_str}, shape={frame.shape}, dtype={frame.dtype}, min={np.min(frame)}, max={np.max(frame)}")
            
            # Update statistics and frame counters
            with self.lock:
                self.frames_received_count += 1
                self.bytes_received_count += len(message_data)
                self.last_frame_time[camera_id_str] = time.time()
                self.frame_count[camera_id_str] += 1
                
                # Store current frames for display
                if camera_id_str == "left":
                    self.current_left_frame = frame.copy()
                else:
                    self.current_right_frame = frame.copy()
                
                # Store latest frame (keep original functionality)
                self.latest_frames[camera_id_str] = {
                    'frame': frame.copy(),
                    'timestamp': timestamp,
                    'received_time': time.time()
                }
                
                # Add to frame buffer for optional saving
                self.frame_buffer.append({
                    'camera': camera_id_str,
                    'frame': frame.copy(),
                    'timestamp': timestamp
                })
                
                # Print stats every 5 seconds with FPS calculation (like UDP receiver)
                current_time = time.time()
                if current_time - self.last_stats_time > STATS_PRINT_INTERVAL_SECONDS:
                    time_diff = current_time - self.last_stats_time
                    left_fps = (self.frame_count["left"] - self.last_frame_count["left"]) / time_diff
                    right_fps = (self.frame_count["right"] - self.last_frame_count["right"]) / time_diff
                    
                    timestamp_str = time.strftime("%H:%M:%S", time.localtime(current_time))
                    print(f"STATUS [{timestamp_str}]: L_FPS={left_fps:.1f}, R_FPS={right_fps:.1f} | Total L={self.frame_count['left']}, R={self.frame_count['right']}")
                    
                    self.last_stats_time = current_time
                    self.last_frame_count = self.frame_count.copy()
            
            # Save frame if enabled
            if SAVE_FRAMES:
                self._save_frame(frame, camera_id_str, timestamp)
                
        except struct.error as e:
            print(f"Struct unpacking error: {e}, message length: {len(message_data)}")
            # Print first few bytes for debugging
            if len(message_data) > 0:
                first_bytes = message_data[:min(20, len(message_data))]
                print(f"First bytes: {first_bytes}")
        except Exception as e:
            print(f"Error processing message: {e}")
            import traceback
            traceback.print_exc()

    def _save_frame(self, frame, camera_id_str, timestamp):
        """Save frame to disk."""
        try:
            filename = f"{OUTPUT_DIR}/frame_{camera_id_str}_{timestamp:.6f}.png"
            cv2.imwrite(filename, frame)
        except Exception as e:
            print(f"Error saving frame: {e}")
            
    def _display_loop(self):
        """Display frames in a single OpenCV window (like UDP receiver)."""
        print("Starting display loop...")
        
        WINDOW_NAME = "Aria SLAM Stream (Direct IPC - Rotated)"
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        
        # Original: (480,640) -> Rotated: (640,480) -> Concatenated: 640x960
        cv2.resizeWindow(WINDOW_NAME, 960, 640) 
        cv2.moveWindow(WINDOW_NAME, 50, 50)
        
        first_pair_received = False
        
        while self.running and DISPLAY_ENABLED:
            try:
                # Get current frames safely
                with self.lock:
                    current_left = self.current_left_frame.copy() if self.current_left_frame is not None else None
                    current_right = self.current_right_frame.copy() if self.current_right_frame is not None else None
                
                if current_left is not None and current_right is not None:
                    if not first_pair_received:
                        first_pair_received = True
                        print("First stereo pair received, starting display...")
                    
                    try:
                        # Apply rotation to both frames (like UDP receiver)
                        rotated_left = np.rot90(current_left, ROTATION_K)
                        rotated_right = np.rot90(current_right, ROTATION_K)

                        # Convert to BGR for display
                        if rotated_left.ndim == 2:
                            left_display = cv2.cvtColor(rotated_left, cv2.COLOR_GRAY2BGR)
                        else:
                            left_display = rotated_left
                        
                        if rotated_right.ndim == 2:
                            right_display = cv2.cvtColor(rotated_right, cv2.COLOR_GRAY2BGR)
                        else:
                            right_display = rotated_right
                        
                        # Handle size alignment after rotation
                        h1, w1 = left_display.shape[:2]
                        h2, w2 = right_display.shape[:2]
                        if h1 != 0 and h2 != 0 and (h1 != h2 or w1 != w2):
                            target_h = max(h1, h2)
                            target_w_l = int(w1 * target_h / h1) if h1 != 0 else 0
                            target_w_r = int(w2 * target_h / h2) if h2 != 0 else 0

                            if h1 != target_h or w1 != target_w_l:
                                left_display = cv2.resize(left_display, (target_w_l, target_h))
                            if h2 != target_h or w2 != target_w_r:
                                right_display = cv2.resize(right_display, (target_w_r, target_h))
                        elif h1 == 0 or h2 == 0:
                            continue

                        # Combine left and right views horizontally
                        combined_view = np.hstack((left_display, right_display))
                        cv2.imshow(WINDOW_NAME, combined_view)
                        
                    except cv2.error as e:
                        print(f"OpenCV error: {e}")
                    except Exception as e:
                        print(f"Display error: {e}")
                
                # Handle key presses
                key = cv2.waitKey(10) & 0xFF
                if key == ord('q'):
                    print("User requested quit via display window")
                    self.running = False
                    break
                elif key == ord('s'):  # 's' to save current frames
                    self._save_current_frames()
                    
            except Exception as e:
                print(f"Error in display loop: {e}")
                
            time.sleep(0.01)  # Small delay
            
        # Cleanup OpenCV windows
        cv2.destroyAllWindows()
        print("Display loop ended")
        
    def _save_current_frames(self):
        """Save current frames manually."""
        with self.lock:
            timestamp = time.time()
            for camera_id_str, frame_data in self.latest_frames.items():
                frame = frame_data['frame']
                filename = f"{OUTPUT_DIR}/manual_save_{camera_id_str}_{timestamp:.6f}.png"
                cv2.imwrite(filename, frame)
                print(f"Saved frame: {filename}")
                
    def get_statistics(self):
        """Get receiver statistics."""
        elapsed_time = time.time() - self.start_time
        avg_fps = self.frames_received_count / elapsed_time if elapsed_time > 1 else self.frames_received_count
        avg_mbps = (self.bytes_received_count * 8 / (1024*1024)) / elapsed_time if elapsed_time > 1 else (self.bytes_received_count * 8 / (1024*1024))
        
        return {
            'frames_received': self.frames_received_count,
            'bytes_received': self.bytes_received_count,
            'runtime': elapsed_time,
            'avg_fps': avg_fps,
            'avg_mbps': avg_mbps,
            'connected': self.connected,
            'last_frame_times': self.last_frame_time.copy()
        }
        
    def close(self):
        """Clean shutdown of receiver."""
        print("Direct IPC Receiver shutting down...")
        self.running = False
        
        # Get final statistics
        stats = self.get_statistics()
        
        print(f"  Total received: {stats['frames_received']} frames, {stats['bytes_received'] / (1024*1024):.2f} MB")
        print(f"  Runtime: {stats['runtime']:.2f} seconds")
        print(f"  Average rate: {stats['avg_fps']:.1f} FPS, {stats['avg_mbps']:.2f} Mbps")
        
        # Close socket
        if self.client_socket:
            try:
                self.client_socket.close()
                print("  Client socket closed.")
            except Exception as e:
                print(f"  Warning: Error closing socket: {e}")
                
        # Wait for threads to finish
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=2.0)
            
        if self.display_thread and self.display_thread.is_alive():
            self.display_thread.join(timeout=2.0)
            
        # Cleanup OpenCV
        if DISPLAY_ENABLED:
            cv2.destroyAllWindows()
            
        print("Direct IPC Receiver shutdown complete.")


# Global cleanup flag
_cleanup_in_progress = False

def signal_handler(signum, frame):
    """Handle signals to ensure proper exit."""
    global _cleanup_in_progress
    if _cleanup_in_progress:
        print("\nForced exit...")
        os._exit(1)
    else:
        print(f"\nReceived signal {signum}, initiating cleanup...")
        _cleanup_in_progress = True

def main():
    """Main execution function for Direct IPC receiver."""
    
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    global _cleanup_in_progress
    
    print("Direct IPC Receiver for Aria SLAM Data")
    print("=" * 40)
    print("Controls:")
    print("  'q' - Quit")
    print("  's' - Save current frames")
    print("  Ctrl+C - Graceful shutdown")
    print("=" * 40)
    
    receiver = None
    
    try:
        # Initialize receiver
        receiver = DirectIPCReceiver(IPC_SOCKET_PATH)
        
        # Attempt to connect
        if not receiver.connect():
            print("Failed to connect to IPC server. Make sure direct_ipc_sender is running.")
            return
            
        print("Connected successfully, receiving data...")
        print(f"Display enabled: {DISPLAY_ENABLED}")
        print(f"Save frames: {SAVE_FRAMES}")
        if SAVE_FRAMES:
            print(f"Output directory: {OUTPUT_DIR}")
            
        # Main loop - much simpler now since stats are printed in _process_message
        while not _cleanup_in_progress and receiver.running:
            # Check if receiver is still connected
            if not receiver.connected:
                print("Lost connection to server, attempting to reconnect...")
                if not receiver.connect():
                    print("Reconnection failed, exiting...")
                    break
                    
            time.sleep(0.1)  # Small delay to prevent excessive CPU usage
            
    except KeyboardInterrupt:
        print("\nUser interrupted (Ctrl+C).")
        _cleanup_in_progress = True
    except Exception as e:
        print(f"Error occurred: {e}")
        import traceback
        traceback.print_exc()
        _cleanup_in_progress = True
    finally:
        _cleanup_in_progress = True
        print("=" * 50)
        print("Shutting down...")
        
        if receiver is not None:
            try:
                receiver.close()
            except Exception as e:
                print(f"Error closing receiver: {e}")
        else:
            print("Receiver was not initialized.")
            
        print("Cleanup complete.")
        print("=" * 50)

if __name__ == "__main__":
    main()
