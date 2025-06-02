# Aria SLAM ZMQ Stereo Receiver
# 
# Receives and displays stereo camera streams via ZeroMQ (ZMQ) IPC.
# Designed to run outside the Docker container and connect to a containerized sender.
# Uses raw (uncompressed) image data for low-latency visualization.
# Supports saving stereo image pairs to disk with timestamped filenames.
# Press 'q' to quit, 's' to save the current stereo pair.
# 
# Author: [Your Name or Organization]
# Date: [Optional]

import struct
import numpy as np
import cv2
import time
import threading
import os
from queue import LifoQueue, Empty

try:
    import zmq
except ImportError:
    print("Error: pyzmq not installed. Install with: pip install pyzmq")
    exit(1)

# ======================= Configuration =======================
ZMQ_CONNECT_ADDRESS = "ipc:///tmp/aria_slam.ipc"  # IPC socket for local communication
ROTATION_K = -1  # np.rot90 k parameter: -1=counterclockwise 90°, 1=clockwise 90°
VERBOSE_LOGGING = False  # Set to True for detailed frame parsing logs
HEARTBEAT_TIMEOUT = 5.0  # Seconds without heartbeat before connection warning
SAVE_BASE_DIR = "/home/vv"  # Base directory for saving images
# =============================================================

def parse_frame_payload(payload: bytes):
    """Parse binary payload to reconstruct frame and metadata."""
    try:
        header_format = '!dBHHB'  # timestamp, cam_id, height, width, channels
        header_size = struct.calcsize(header_format)
        if len(payload) < header_size: 
            return None
        
        header_data = payload[:header_size]
        timestamp, cam_id_byte, height, width, channels = struct.unpack(header_format, header_data)
        
        frame_data = payload[header_size:]
        expected_size = height * width * channels
        if len(frame_data) != expected_size:
            print(f"Incomplete data! H={height},W={width},C={channels} -> Expected:{expected_size}, Actual:{len(frame_data)}")
            return None

        shape = (height, width, channels) if channels > 1 else (height, width)
        frame = np.frombuffer(frame_data, dtype=np.uint8).reshape(shape)
        camera_id_str = "left" if cam_id_byte == 1 else "right"
        
        # Only log detailed info if verbose logging is enabled
        if VERBOSE_LOGGING:
            print(f"Frame parsed: {camera_id_str}, shape={frame.shape}, dtype={frame.dtype}, min={np.min(frame)}, max={np.max(frame)}")
        
        return {"id": camera_id_str, "frame": frame, "ts": timestamp}
    except Exception as e:
        print(f"Error parsing payload: {e}")
        return None


def zmq_receive_thread(context, connect_address, stereo_pair_queue, running_flag, connection_status):
    """ZeroMQ receiver thread - subscribes to SLAM topics and assembles stereo pairs."""
    
    socket = context.socket(zmq.SUB)
    socket.connect(connect_address)
    
    # Subscribe to SLAM camera topics and heartbeat
    socket.setsockopt(zmq.SUBSCRIBE, b"slam.left")
    socket.setsockopt(zmq.SUBSCRIBE, b"slam.right") 
    socket.setsockopt(zmq.SUBSCRIBE, b"heartbeat")
    
    # Set socket timeout for non-blocking receives
    socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1 second timeout
    
    print(f"ZMQ receiver connected to: {connect_address}")
    print("Subscribed to topics: slam.left, slam.right, heartbeat")
    
    temp_frames_buffer = {}
    frame_count = {"left": 0, "right": 0}
    last_stats_time = time.time()
    last_frame_count = {"left": 0, "right": 0}
    last_heartbeat_time = time.time()
    
    while running_flag.is_set():
        try:
            # Receive message with topic
            topic, payload = socket.recv_multipart()
            topic_str = topic.decode('utf-8')
            current_time = time.time()
            
            if topic_str == "heartbeat":
                # Process heartbeat message
                if len(payload) >= 8:
                    sender_timestamp = struct.unpack('!d', payload)[0]
                    last_heartbeat_time = current_time
                    connection_status['last_heartbeat'] = current_time
                    connection_status['connected'] = True
                    
                    if VERBOSE_LOGGING:
                        print(f"Heartbeat received, sender time: {sender_timestamp:.3f}")
                
            elif topic_str.startswith("slam."):
                # Process frame data
                parsed_data = parse_frame_payload(payload)
                if parsed_data:
                    camera_id = parsed_data['id']
                    frame_count[camera_id] += 1
                    temp_frames_buffer[camera_id] = parsed_data
                    
                    # Update connection status
                    connection_status['last_frame'] = current_time
                    connection_status['connected'] = True
                    
                    # Print statistics every 5 seconds with FPS calculation
                    if current_time - last_stats_time > 5.0:
                        time_diff = current_time - last_stats_time
                        left_fps = (frame_count["left"] - last_frame_count["left"]) / time_diff
                        right_fps = (frame_count["right"] - last_frame_count["right"]) / time_diff
                        
                        timestamp = time.strftime("%H:%M:%S", time.localtime(current_time))
                        print(f"[{timestamp}] Received FPS: Left={left_fps:.1f}, Right={right_fps:.1f} | "
                              f"Total frames: L={frame_count['left']} R={frame_count['right']}")
                        
                        last_frame_count = frame_count.copy()
                        last_stats_time = current_time
                    
                    # Check if we have both left and right frames for stereo pair
                    if 'left' in temp_frames_buffer and 'right' in temp_frames_buffer:
                        stereo_data = {
                            "left_frame": temp_frames_buffer["left"]["frame"],
                            "right_frame": temp_frames_buffer["right"]["frame"],
                            "left_timestamp": temp_frames_buffer["left"]["ts"],
                            "right_timestamp": temp_frames_buffer["right"]["ts"]
                        }
                        
                        # Try to put stereo pair in queue (non-blocking)
                        try:
                            stereo_pair_queue.put_nowait(stereo_data)
                        except:
                            # Queue is full, discard oldest and add new
                            try:
                                stereo_pair_queue.get_nowait()
                                stereo_pair_queue.put_nowait(stereo_data)
                            except:
                                pass
                        
                        # Clear buffer after creating stereo pair
                        temp_frames_buffer.clear()
            
            # Check for connection timeout
            if current_time - last_heartbeat_time > HEARTBEAT_TIMEOUT:
                if connection_status['connected']:
                    print(f"Warning: No heartbeat received for {HEARTBEAT_TIMEOUT} seconds")
                    connection_status['connected'] = False
                    
        except zmq.Again:
            # Timeout occurred, check connection status
            current_time = time.time()
            if current_time - last_heartbeat_time > HEARTBEAT_TIMEOUT:
                if connection_status['connected']:
                    print("Connection timeout - waiting for sender...")
                    connection_status['connected'] = False
            continue
            
        except Exception as e:
            if running_flag.is_set():
                print(f"ZMQ receiver thread error: {e}")
    
    socket.close()
    print("ZMQ receiver thread stopped.")


def save_stereo_images(left_frame, right_frame):
    """Save stereo images to Aria_image folder structure."""
    try:
        # Create timestamp for filename
        timestamp = time.strftime("%Y%m%d_%H%M%S_%f", time.localtime())[:-3]  # Remove last 3 digits of microseconds
        
        # Create folder structure: Aria_image/slam_left and Aria_image/slam_right
        aria_folder = os.path.join(SAVE_BASE_DIR, "Aria_image")
        left_folder = os.path.join(aria_folder, "slam_left")
        right_folder = os.path.join(aria_folder, "slam_right")
        
        # Create directories
        os.makedirs(left_folder, exist_ok=True)
        os.makedirs(right_folder, exist_ok=True)
        
        # Save images with timestamp filename
        left_path = os.path.join(left_folder, f"{timestamp}.png")
        right_path = os.path.join(right_folder, f"{timestamp}.png")
        
        cv2.imwrite(left_path, left_frame)
        cv2.imwrite(right_path, right_frame)
        
        print(f"Images saved: {left_path} and {right_path}")
        return True
    except Exception as e:
        print(f"Error saving images: {e}")
        return False

def main():
    """Main function for ZMQ receiver application."""
    
    # Initialize ZeroMQ context
    context = zmq.Context()
    
    # Create queue for stereo pairs and connection status
    stereo_pair_queue = LifoQueue(maxsize=1)
    running_flag = threading.Event()
    running_flag.set()
    
    connection_status = {
        'connected': False,
        'last_heartbeat': 0,
        'last_frame': 0
    }

    # Start ZMQ receiver thread
    receiver_thread = threading.Thread(
        target=zmq_receive_thread, 
        args=(context, ZMQ_CONNECT_ADDRESS, stereo_pair_queue, running_flag, connection_status)
    )
    receiver_thread.daemon = True
    receiver_thread.start()

    # Initialize display variables
    current_left_img_raw = None
    current_right_img_raw = None
    
    WINDOW_NAME = "Aria SLAM Stream (ZMQ - Rotated)"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    
    # Set window size for rotated images
    # Original: (480,640) -> Rotated: (640,480) -> Concatenated: 640x960
    cv2.resizeWindow(WINDOW_NAME, 960, 640) 
    cv2.moveWindow(WINDOW_NAME, 50, 50)

    first_pair_received = False
    connection_warning_shown = False

    try:
        print("ZMQ receiver started. Press 'q' to quit, 's' to save current stereo pair.")
        
        while True:
            # Try to get latest stereo pair from queue
            try:
                stereo_pair_data = stereo_pair_queue.get_nowait()
                current_left_img_raw = stereo_pair_data["left_frame"]
                current_right_img_raw = stereo_pair_data["right_frame"]
                
                if not first_pair_received:
                    first_pair_received = True
                    print("First stereo pair received, starting display...")
                    
            except Empty:
                pass

            # Display stereo pair if available
            if current_left_img_raw is not None and current_right_img_raw is not None:
                try:
                    # Apply rotation to both frames for proper orientation
                    left_rotated = np.rot90(current_left_img_raw, k=ROTATION_K)
                    right_rotated = np.rot90(current_right_img_raw, k=ROTATION_K)
                    
                    # Concatenate frames horizontally for side-by-side display
                    stereo_display = np.hstack([left_rotated, right_rotated])
                    
                    # Ensure the array is contiguous for OpenCV display
                    if not stereo_display.flags['C_CONTIGUOUS']:
                        stereo_display = np.ascontiguousarray(stereo_display)
                    
                    cv2.imshow(WINDOW_NAME, stereo_display)
                    
                    # Update connection status
                    if not connection_warning_shown:
                        connection_warning_shown = False
                    
                except cv2.error as e:
                    print(f"OpenCV error during display: {e}")
                except Exception as e:
                    print(f"Error during image processing: {e}")
            else:
                # Show black screen when no frames available
                waiting_img = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.imshow(WINDOW_NAME, waiting_img)
                
                # Print connection status to console instead
                if not connection_status['connected'] and not connection_warning_shown:
                    print("Waiting for connection to ZMQ sender...")
                    connection_warning_shown = True
                elif connection_status['connected'] and connection_warning_shown:
                    print("Connected! Waiting for frames...")
                    connection_warning_shown = False

            # Check for quit key and save key
            key = cv2.waitKey(10) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s') and current_left_img_raw is not None and current_right_img_raw is not None:
                # Save current stereo pair
                left_rotated = np.rot90(current_left_img_raw, k=ROTATION_K)
                right_rotated = np.rot90(current_right_img_raw, k=ROTATION_K)
                save_stereo_images(left_rotated, right_rotated)

    except KeyboardInterrupt:
        print("\nUser interrupted.")
    finally:
        print("Cleaning up ZMQ receiver...")
        running_flag.clear()
        
        # Wait for receiver thread to finish
        if receiver_thread.is_alive():
            receiver_thread.join(timeout=2.0)
        
        # Clean up ZMQ context and OpenCV
        context.term()
        cv2.destroyAllWindows()
        print("ZMQ receiver closed.")


if __name__ == "__main__":
    main()
