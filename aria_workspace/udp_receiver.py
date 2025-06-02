# UDP Receiver for Aria SLAM - Receives and decompresses images

import socket
import struct
import numpy as np
import cv2
import time
import threading
from queue import LifoQueue, Empty

# ======================= Configuration =======================
LISTEN_IP = "127.0.0.1"
LISTEN_PORT = 9999
BUFFER_SIZE = 65536
ROTATION_K = -1  # np.rot90 k parameter: -1=counterclockwise 90°, 1=clockwise 90°
VERBOSE_LOGGING = False  # Set to True for detailed frame parsing logs
# =============================================================

class FrameReassembler:
    """Frame reassembler for handling fragmented UDP packets."""
    def __init__(self):
        self.assembly_buffer = {}

    def process_packet(self, packet: bytes):
        magic_number = packet[:4]
        if magic_number == b'COMP':
            # Compressed single packet
            return packet[4:]
        elif magic_number == b'SNGL':
            return packet[4:]
        elif magic_number == b'FRAG':
            try:
                header_format = '!IHH'  # packet_id, chunk_index, total_chunks
                header_size = struct.calcsize(header_format)
                if len(packet) < (4 + header_size):
                    return None

                header_data = packet[4 : 4 + header_size]
                chunk_data = packet[4 + header_size :]
                packet_id, chunk_index, total_chunks = struct.unpack(header_format, header_data)
                
                if packet_id not in self.assembly_buffer:
                    if total_chunks == 0:
                        return None
                    self.assembly_buffer[packet_id] = [None] * total_chunks
                
                if chunk_index < len(self.assembly_buffer[packet_id]):
                    self.assembly_buffer[packet_id][chunk_index] = chunk_data
                else:
                    if packet_id in self.assembly_buffer: del self.assembly_buffer[packet_id]
                    return None

                if all(chunk is not None for chunk in self.assembly_buffer[packet_id]):
                    full_payload = b"".join(self.assembly_buffer[packet_id])
                    del self.assembly_buffer[packet_id]
                    return full_payload
            except Exception as e:
                 if 'packet_id' in locals() and packet_id in self.assembly_buffer:
                     del self.assembly_buffer[packet_id]
                 return None
        return None

def parse_frame_payload(payload: bytes):
    """Parse binary payload to reconstruct frame and metadata (supports both compressed and uncompressed)."""
    try:
        # Try compressed format first: timestamp, cam_id, orig_height, orig_width, orig_channels, compressed_size
        compressed_header_format = '!dBHHBI'
        compressed_header_size = struct.calcsize(compressed_header_format)
        
        if len(payload) >= compressed_header_size:
            header_data = payload[:compressed_header_size]
            timestamp, cam_id_byte, height, width, channels, compressed_size = struct.unpack(compressed_header_format, header_data)
            
            # Check if this looks like compressed data
            if compressed_size > 0 and compressed_size == len(payload) - compressed_header_size:
                # This is compressed data
                compressed_data = payload[compressed_header_size:]
                
                # Decompress JPEG data
                try:
                    compressed_array = np.frombuffer(compressed_data, dtype=np.uint8)
                    decoded_frame = cv2.imdecode(compressed_array, cv2.IMREAD_COLOR)
                    
                    if decoded_frame is None:
                        print(f"Failed to decode JPEG data")
                        return None
                    
                    # Convert back to original format if needed
                    if channels == 1:
                        frame = cv2.cvtColor(decoded_frame, cv2.COLOR_BGR2GRAY)
                    else:
                        frame = decoded_frame
                    
                    camera_id_str = "left" if cam_id_byte == 1 else "right"
                    
                    if VERBOSE_LOGGING:
                        print(f"Compressed frame parsed: {camera_id_str}, original=({height},{width},{channels}), "
                              f"decompressed={frame.shape}, compressed_size={compressed_size}")
                    
                    return {"id": camera_id_str, "frame": frame, "ts": timestamp}
                    
                except Exception as e:
                    print(f"Error decompressing JPEG data: {e}")
                    return None
        
        # Fall back to uncompressed format
        header_format = '!dBHHB'  # timestamp, cam_id, height, width, channels
        header_size = struct.calcsize(header_format)
        if len(payload) < header_size: 
            return None
        
        header_data = payload[:header_size]
        timestamp, cam_id_byte, height, width, channels = struct.unpack(header_format, header_data)
        
        frame_data = payload[header_size:]
        expected_size = height * width * channels
        if len(frame_data) != expected_size:
            print(f"Incomplete uncompressed data! H={height},W={width},C={channels} -> Expected:{expected_size}, Actual:{len(frame_data)}")
            return None

        shape = (height, width, channels) if channels > 1 else (height, width)
        frame = np.frombuffer(frame_data, dtype=np.uint8).reshape(shape)
        camera_id_str = "left" if cam_id_byte == 1 else "right"
        
        if VERBOSE_LOGGING:
            print(f"Uncompressed frame parsed: {camera_id_str}, shape={frame.shape}, dtype={frame.dtype}")
        
        return {"id": camera_id_str, "frame": frame, "ts": timestamp}
    except Exception as e:
        print(f"Error parsing payload: {e}")
        return None

def receive_thread_func(sock, stereo_pair_queue, reassembler, running_flag):
    """Network receiver thread - processes packets and assembles stereo pairs."""
    temp_frames_buffer = {} 
    frame_count = {"left": 0, "right": 0}
    last_stats_time = time.time()
    last_frame_count = {"left": 0, "right": 0}
    
    while running_flag.is_set():
        try:
            packet, _ = sock.recvfrom(BUFFER_SIZE)
            full_payload = reassembler.process_packet(packet)
            
            if full_payload:
                parsed_data = parse_frame_payload(full_payload)
                if parsed_data:
                    camera_id = parsed_data['id']
                    frame_count[camera_id] += 1
                    temp_frames_buffer[camera_id] = parsed_data
                    
                    # Print stats every 5 seconds with FPS calculation
                    current_time = time.time()
                    if current_time - last_stats_time > 5.0:
                        time_diff = current_time - last_stats_time
                        left_fps = (frame_count["left"] - last_frame_count["left"]) / time_diff
                        right_fps = (frame_count["right"] - last_frame_count["right"]) / time_diff
                        
                        timestamp = time.strftime("%H:%M:%S", time.localtime(current_time))
                        print(f"STATUS [{timestamp}]: L_FPS={left_fps:.1f}, R_FPS={right_fps:.1f} | Total L={frame_count['left']}, R={frame_count['right']}")
                        
                        last_stats_time = current_time
                        last_frame_count = frame_count.copy()
                    
                    if 'left' in temp_frames_buffer and 'right' in temp_frames_buffer:
                        left_data = temp_frames_buffer.pop('left')
                        right_data = temp_frames_buffer.pop('right')
                        stereo_pair_queue.put({
                            "left_frame": left_data["frame"],
                            "right_frame": right_data["frame"],
                        })
        except socket.timeout:
            continue
        except Exception as e:
            if running_flag.is_set():
                print(f"Receiver thread error: {e}")

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    print(f"Aria UDP Receiver started, listening on {LISTEN_IP}:{LISTEN_PORT}...")

    reassembler = FrameReassembler()
    stereo_pair_queue = LifoQueue(maxsize=1) 
    running_flag = threading.Event()
    running_flag.set()

    receiver = threading.Thread(
        target=receive_thread_func, 
        args=(sock, stereo_pair_queue, reassembler, running_flag)
    )
    receiver.daemon = True
    receiver.start()

    current_left_img_raw = None
    current_right_img_raw = None
    
    WINDOW_NAME = "Aria SLAM Stream (Rotated)"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    
    # Original: (480,640) -> Rotated: (640,480) -> Concatenated: 640x960
    cv2.resizeWindow(WINDOW_NAME, 960, 640) 
    cv2.moveWindow(WINDOW_NAME, 50, 50)

    first_pair_received = False

    try:
        while True:
            try:
                stereo_pair_data = stereo_pair_queue.get_nowait() 
                current_left_img_raw = stereo_pair_data["left_frame"]
                current_right_img_raw = stereo_pair_data["right_frame"]
                if not first_pair_received:
                    first_pair_received = True
                    print("First stereo pair received, starting display...")
            except Empty:
                pass

            if current_left_img_raw is not None and current_right_img_raw is not None:
                try:
                    # Apply rotation to both frames
                    rotated_left = np.rot90(current_left_img_raw, ROTATION_K)
                    rotated_right = np.rot90(current_right_img_raw, ROTATION_K)

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
                    if h1 !=0 and h2 != 0 and (h1 != h2 or w1 != w2):
                        target_h = max(h1, h2)
                        target_w_l = int(w1 * target_h / h1) if h1 != 0 else 0
                        target_w_r = int(w2 * target_h / h2) if h2 != 0 else 0

                        if h1 != target_h or w1 != target_w_l :
                             left_display = cv2.resize(left_display, (target_w_l, target_h))
                        if h2 != target_h or w2 != target_w_r:
                             right_display = cv2.resize(right_display, (target_w_r, target_h))
                    elif h1 == 0 or h2 == 0:
                        continue

                    combined_view = np.hstack((left_display, right_display))
                    cv2.imshow(WINDOW_NAME, combined_view)
                except cv2.error as e:
                    print(f"OpenCV error: {e}")
                except Exception as e:
                    print(f"Display error: {e}")
            
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nUser interrupted.")
    finally:
        print("Cleaning up...")
        running_flag.clear()
        if receiver.is_alive():
            receiver.join(timeout=1.0)
        sock.close()
        cv2.destroyAllWindows()
        print("Receiver closed.")

if __name__ == "__main__":
    main()