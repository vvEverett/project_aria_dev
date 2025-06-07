# Aria SLAM + RGB ZMQ Receiver
# 
# Receives and displays stereo camera streams + RGB camera via ZeroMQ (ZMQ) IPC.
# Designed to run outside the Docker container and connect to a containerized sender.
# Uses raw (uncompressed) image data for low-latency visualization.
# Supports saving stereo image pairs and RGB images to disk with timestamped filenames.
# Press 'q' to quit, 's' to save the current images.

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
ENABLE_RGB_DISPLAY = True  # Set to True to display RGB camera
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
        
        # Updated camera ID mapping to handle RGB
        if cam_id_byte == 1:
            camera_id_str = "left"
        elif cam_id_byte == 2:
            camera_id_str = "right"
        elif cam_id_byte == 3:
            camera_id_str = "rgb"
        else:
            camera_id_str = f"unknown_{cam_id_byte}"
        
        # Only log detailed info if verbose logging is enabled
        if VERBOSE_LOGGING:
            print(f"Frame parsed: {camera_id_str}, shape={frame.shape}, dtype={frame.dtype}, min={np.min(frame)}, max={np.max(frame)}")
        
        return {"id": camera_id_str, "frame": frame, "ts": timestamp}
    except Exception as e:
        print(f"Error parsing payload: {e}")
        return None


def zmq_receive_thread(context, connect_address, frame_queue, running_flag, connection_status):
    """ZeroMQ receiver thread - subscribes to SLAM and RGB topics."""
    
    socket = context.socket(zmq.SUB)
    socket.connect(connect_address)
    
    # Subscribe to SLAM camera topics, RGB topic, and heartbeat
    socket.setsockopt(zmq.SUBSCRIBE, b"slam.left")
    socket.setsockopt(zmq.SUBSCRIBE, b"slam.right")
    socket.setsockopt(zmq.SUBSCRIBE, b"slam.rgb")  # Subscribe to RGB topic
    socket.setsockopt(zmq.SUBSCRIBE, b"heartbeat")
    
    # Set socket timeout for non-blocking receives
    socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1 second timeout
    
    print(f"ZMQ receiver connected to: {connect_address}")
    print("Subscribed to topics: slam.left, slam.right, slam.rgb, heartbeat")
    
    temp_frames_buffer = {}
    frame_count = {"left": 0, "right": 0, "rgb": 0}
    last_stats_time = time.time()
    last_frame_count = {"left": 0, "right": 0, "rgb": 0}
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
                        rgb_fps = (frame_count["rgb"] - last_frame_count["rgb"]) / time_diff
                        
                        timestamp = time.strftime("%H:%M:%S", time.localtime(current_time))
                        print(f"[{timestamp}] Received FPS: Left={left_fps:.1f}, Right={right_fps:.1f}, RGB={rgb_fps:.1f} | "
                              f"Total frames: L={frame_count['left']} R={frame_count['right']} RGB={frame_count['rgb']}")
                        
                        last_frame_count = frame_count.copy()
                        last_stats_time = current_time
                    
                    # Put individual frames in queue for display
                    frame_data = {
                        "camera_id": camera_id,
                        "frame": parsed_data["frame"],
                        "timestamp": parsed_data["ts"]
                    }
                    
                    # Try to put frame in queue (non-blocking)
                    try:
                        frame_queue.put_nowait(frame_data)
                    except:
                        # Queue is full, discard oldest and add new
                        try:
                            frame_queue.get_nowait()
                            frame_queue.put_nowait(frame_data)
                        except:
                            pass
            
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


def save_images(left_frame=None, right_frame=None, rgb_frame=None):
    """Save images to Aria_image folder structure."""
    try:
        # Create timestamp for filename
        timestamp = time.strftime("%Y%m%d_%H%M%S_%f", time.localtime())[:-3]  # Remove last 3 digits of microseconds
        
        # Create folder structure
        aria_folder = os.path.join(SAVE_BASE_DIR, "Aria_image")
        folders = {
            "slam_left": os.path.join(aria_folder, "slam_left"),
            "slam_right": os.path.join(aria_folder, "slam_right"),
            "rgb": os.path.join(aria_folder, "rgb")  # RGB is now grayscale
        }
        
        # Create directories
        for folder in folders.values():
            os.makedirs(folder, exist_ok=True)
        
        saved_files = []
        
        # Save images with timestamp filename
        if left_frame is not None:
            left_path = os.path.join(folders["slam_left"], f"{timestamp}.png")
            cv2.imwrite(left_path, left_frame)
            saved_files.append(left_path)
        
        if right_frame is not None:
            right_path = os.path.join(folders["slam_right"], f"{timestamp}.png")
            cv2.imwrite(right_path, right_frame)
            saved_files.append(right_path)
        
        if rgb_frame is not None:
            rgb_path = os.path.join(folders["rgb"], f"{timestamp}.png")
            cv2.imwrite(rgb_path, rgb_frame)  # OpenCV handles grayscale automatically
            saved_files.append(rgb_path)
        
        if saved_files:
            print(f"Images saved: {', '.join(saved_files)}")
            return True
    except Exception as e:
        print(f"Error saving images: {e}")
    return False

def main():
    """Main function for ZMQ receiver application."""
    
    # Initialize ZeroMQ context
    context = zmq.Context()
    
    # Create queue for frames and connection status
    frame_queue = LifoQueue(maxsize=10)  # Larger queue for multiple cameras
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
        args=(context, ZMQ_CONNECT_ADDRESS, frame_queue, running_flag, connection_status)
    )
    receiver_thread.daemon = True
    receiver_thread.start()

    # Initialize display variables
    current_frames = {"left": None, "right": None, "rgb": None}
    
    # Setup windows
    STEREO_WINDOW = "Aria SLAM Stream (ZMQ - Rotated)"
    RGB_WINDOW = "Aria RGB Stream (ZMQ - Rotated)"  # Updated window title
    
    cv2.namedWindow(STEREO_WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(STEREO_WINDOW, 960, 640)  # Concatenated stereo
    cv2.moveWindow(STEREO_WINDOW, 50, 50)
    
    if ENABLE_RGB_DISPLAY:
        cv2.namedWindow(RGB_WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(RGB_WINDOW, 480, 640)  # RGB image size
        cv2.moveWindow(RGB_WINDOW, 1050, 50)

    first_frame_received = {"left": False, "right": False, "rgb": False}
    connection_warning_shown = False

    try:
        print("ZMQ receiver started. Press 'q' to quit, 's' to save current images.")
        print("Note: RGB camera now sends grayscale images for reduced bandwidth.")
        
        while True:
            # Process frames from queue
            try:
                while True:
                    frame_data = frame_queue.get_nowait()
                    camera_id = frame_data["camera_id"]
                    current_frames[camera_id] = frame_data["frame"]
                    
                    if not first_frame_received[camera_id]:
                        first_frame_received[camera_id] = True
                        print(f"First {camera_id} frame received...")
                        
            except Empty:
                pass

            # Display stereo pair if available
            if current_frames["left"] is not None and current_frames["right"] is not None:
                try:
                    # Apply rotation to both frames for proper orientation
                    left_rotated = np.rot90(current_frames["left"], k=ROTATION_K)
                    right_rotated = np.rot90(current_frames["right"], k=ROTATION_K)
                    
                    # Concatenate frames horizontally for side-by-side display
                    stereo_display = np.hstack([left_rotated, right_rotated])
                    
                    # Ensure the array is contiguous for OpenCV display
                    if not stereo_display.flags['C_CONTIGUOUS']:
                        stereo_display = np.ascontiguousarray(stereo_display)
                    
                    cv2.imshow(STEREO_WINDOW, stereo_display)
                    
                except cv2.error as e:
                    print(f"OpenCV error during stereo display: {e}")
                except Exception as e:
                    print(f"Error during stereo image processing: {e}")
            else:
                # Show black screen when no stereo frames available
                waiting_img = np.zeros((640, 960, 3), dtype=np.uint8)  # Rotated stereo size
                cv2.imshow(STEREO_WINDOW, waiting_img)
            
            # Display RGB frame if available and enabled (now grayscale)
            if ENABLE_RGB_DISPLAY and current_frames["rgb"] is not None:
                try:
                    # RGB frame is now grayscale, apply rotation
                    rgb_rotated = np.rot90(current_frames["rgb"], k=ROTATION_K)
                    
                    # Handle both grayscale and potential color images
                    if rgb_rotated.ndim == 2:
                        # Grayscale image - display directly
                        rgb_display = rgb_rotated
                    elif rgb_rotated.ndim == 3 and rgb_rotated.shape[2] == 3:
                        # Color RGB image - convert to BGR for OpenCV display
                        rgb_display = cv2.cvtColor(rgb_rotated, cv2.COLOR_RGB2BGR)
                    else:
                        # Handle single channel as grayscale
                        rgb_display = rgb_rotated.squeeze()
                    
                    # Ensure the array is contiguous for OpenCV display
                    if not rgb_display.flags['C_CONTIGUOUS']:
                        rgb_display = np.ascontiguousarray(rgb_display)
                    
                    cv2.imshow(RGB_WINDOW, rgb_display)
                    
                except cv2.error as e:
                    print(f"OpenCV error during RGB display: {e}")
                except Exception as e:
                    print(f"Error during RGB image processing: {e}")
            elif ENABLE_RGB_DISPLAY:
                # Show black screen when no RGB frame available
                waiting_rgb = np.zeros((512, 512), dtype=np.uint8)  # Grayscale waiting image
                cv2.imshow(RGB_WINDOW, waiting_rgb)
            
            # Handle connection status
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
            elif key == ord('s'):
                # Save current images
                left_to_save = np.rot90(current_frames["left"], k=ROTATION_K) if current_frames["left"] is not None else None
                right_to_save = np.rot90(current_frames["right"], k=ROTATION_K) if current_frames["right"] is not None else None
                rgb_to_save = None
                
                if current_frames["rgb"] is not None:
                    rgb_rotated = np.rot90(current_frames["rgb"], k=ROTATION_K)
                    # RGB is now grayscale, no color conversion needed
                    rgb_to_save = rgb_rotated
                
                save_images(left_to_save, right_to_save, rgb_to_save)

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
