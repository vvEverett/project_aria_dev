#!/usr/bin/env python3
# Aria SLAM UDP Stereo Receiver (ROS1 Node)
#
# Receives stereo camera streams over UDP, supporting both compressed (JPEG) and uncompressed image formats.
# Handles fragmented UDP packets and reassembles them into complete frames.
# Publishes: /aria/left/image_raw, /aria/right/image_raw (sensor_msgs/Image)
# Optional OpenCV display controlled by enable_cv_display parameter.

import socket
import struct
import numpy as np
import cv2
import time
import threading
import os
from queue import LifoQueue, Empty
from dataclasses import dataclass
from typing import Optional

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

@dataclass
class Config:
    """Centralized configuration class."""
    # UDP settings
    listen_ip: str = "127.0.0.1"
    listen_port: int = 9999
    buffer_size: int = 65536
    socket_timeout: float = 1.0
    
    # Image processing
    rotation_k: int = -1  # np.rot90 k parameter
    
    # Performance settings
    queue_maxsize: int = 1
    stats_interval: float = 5.0
    verbose_logging: bool = False
    
    # Display settings
    enable_cv_display: bool = True
    window_name: str = "Aria SLAM Stream (UDP-ROS)"
    window_width: int = 960
    window_height: int = 640
    window_x: int = 50
    window_y: int = 50
    
    # ROS settings
    ros_queue_size: int = 10
    left_topic: str = "/aria/left/image_raw"
    right_topic: str = "/aria/right/image_raw"
    left_frame_id: str = "aria_left"
    right_frame_id: str = "aria_right"

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
                    if packet_id in self.assembly_buffer: 
                        del self.assembly_buffer[packet_id]
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

class AriaUDPReceiver:
    """High-performance Aria UDP to ROS converter."""
    
    def __init__(self, config: Config):
        self.config = config
        self.stereo_queue = LifoQueue(maxsize=config.queue_maxsize)
        self.running = threading.Event()
        self.reassembler = FrameReassembler()
        
        # ROS components
        self.bridge = CvBridge()
        self.left_pub = rospy.Publisher(config.left_topic, Image, queue_size=config.ros_queue_size)
        self.right_pub = rospy.Publisher(config.right_topic, Image, queue_size=config.ros_queue_size)
        
        # State tracking
        self.first_pair_received = False
        self.current_frames = {'left': None, 'right': None}
        
        # Pre-allocate commonly used objects
        self.waiting_img = np.zeros((480, 640, 3), dtype=np.uint8)
    
    def parse_frame_payload(self, payload: bytes) -> Optional[dict]:
        """Parse binary payload to reconstruct frame and metadata (supports both compressed and uncompressed)."""
        try:
            # Try compressed format first
            compressed_header_format = '!dBHHBI'
            compressed_header_size = struct.calcsize(compressed_header_format)
            
            if len(payload) >= compressed_header_size:
                header_data = payload[:compressed_header_size]
                timestamp, cam_id_byte, height, width, channels, compressed_size = struct.unpack(
                    compressed_header_format, header_data
                )
                
                # Check if this looks like compressed data
                if compressed_size > 0 and compressed_size == len(payload) - compressed_header_size:
                    # This is compressed data
                    compressed_data = payload[compressed_header_size:]
                    
                    # Decompress JPEG data
                    try:
                        compressed_array = np.frombuffer(compressed_data, dtype=np.uint8)
                        decoded_frame = cv2.imdecode(compressed_array, cv2.IMREAD_COLOR)
                        
                        if decoded_frame is None:
                            if self.config.verbose_logging:
                                print(f"Failed to decode JPEG data")
                            return None
                        
                        # Convert back to original format if needed
                        if channels == 1:
                            frame = cv2.cvtColor(decoded_frame, cv2.COLOR_BGR2GRAY)
                        else:
                            frame = decoded_frame
                        
                        camera_id_str = "left" if cam_id_byte == 1 else "right"
                        
                        if self.config.verbose_logging:
                            print(f"Compressed frame parsed: {camera_id_str}, original=({height},{width},{channels}), "
                                  f"decompressed={frame.shape}, compressed_size={compressed_size}")
                        
                        return {"id": camera_id_str, "frame": frame, "ts": timestamp}
                        
                    except Exception as e:
                        if self.config.verbose_logging:
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
                if self.config.verbose_logging:
                    print(f"Incomplete uncompressed data! H={height},W={width},C={channels} -> Expected:{expected_size}, Actual:{len(frame_data)}")
                return None

            shape = (height, width, channels) if channels > 1 else (height, width)
            frame = np.frombuffer(frame_data, dtype=np.uint8).reshape(shape)
            camera_id_str = "left" if cam_id_byte == 1 else "right"
            
            if self.config.verbose_logging:
                print(f"Uncompressed frame parsed: {camera_id_str}, shape={frame.shape}, dtype={frame.dtype}")
            
            return {"id": camera_id_str, "frame": frame, "ts": timestamp}
            
        except Exception as e:
            if self.config.verbose_logging:
                print(f"Error parsing payload: {e}")
            return None
    
    def udp_receive_thread(self):
        """UDP receiver thread - processes packets and assembles stereo pairs."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(self.config.socket_timeout)
        sock.bind((self.config.listen_ip, self.config.listen_port))
        
        print(f"UDP receiver listening on {self.config.listen_ip}:{self.config.listen_port}...")
        
        # Efficient state tracking
        temp_buffer = {}
        frame_counts = {"left": 0, "right": 0}
        last_stats_time = time.time()
        last_frame_counts = {"left": 0, "right": 0}
        
        self.running.set()
        
        while self.running.is_set() and not rospy.is_shutdown():
            try:
                packet, _ = sock.recvfrom(self.config.buffer_size)
                full_payload = self.reassembler.process_packet(packet)
                
                if full_payload:
                    parsed_data = self.parse_frame_payload(full_payload)
                    if parsed_data:
                        camera_id = parsed_data['id']
                        frame_counts[camera_id] += 1
                        temp_buffer[camera_id] = parsed_data
                        
                        # Stats reporting
                        current_time = time.time()
                        if current_time - last_stats_time > self.config.stats_interval:
                            self._print_stats(frame_counts, last_frame_counts, 
                                            current_time - last_stats_time, current_time)
                            last_frame_counts = frame_counts.copy()
                            last_stats_time = current_time
                        
                        # Stereo pair assembly
                        if len(temp_buffer) == 2:  # Both left and right available
                            stereo_data = {
                                "left_frame": temp_buffer["left"]["frame"],
                                "right_frame": temp_buffer["right"]["frame"],
                                "left_timestamp": temp_buffer["left"]["ts"],
                                "right_timestamp": temp_buffer["right"]["ts"]
                            }
                            
                            # Non-blocking queue operation
                            try:
                                self.stereo_queue.put_nowait(stereo_data)
                            except:
                                try:
                                    self.stereo_queue.get_nowait()
                                    self.stereo_queue.put_nowait(stereo_data)
                                except:
                                    pass
                            
                            temp_buffer.clear()
                            
            except socket.timeout:
                continue
            except Exception as e:
                if self.running.is_set():
                    print(f"UDP receiver error: {e}")
        
        sock.close()
        print("UDP receiver thread stopped.")
    
    def _print_stats(self, frame_counts: dict, last_counts: dict, 
                    time_diff: float, current_time: float):
        """Print performance statistics."""
        left_fps = (frame_counts["left"] - last_counts["left"]) / time_diff
        right_fps = (frame_counts["right"] - last_counts["right"]) / time_diff
        timestamp = time.strftime("%H:%M:%S", time.localtime(current_time))
        
        print(f"[{timestamp}] FPS: L={left_fps:.1f}, R={right_fps:.1f} | "
              f"Total: L={frame_counts['left']}, R={frame_counts['right']}")
    
    def publish_to_ros(self, left_frame: np.ndarray, right_frame: np.ndarray, 
                      left_ts: float, right_ts: float):
        """Efficient ROS publishing."""
        try:
            # Create and publish messages
            left_msg = self.bridge.cv2_to_imgmsg(
                left_frame, encoding="bgr8" if left_frame.ndim == 3 else "mono8"
            )
            right_msg = self.bridge.cv2_to_imgmsg(
                right_frame, encoding="bgr8" if right_frame.ndim == 3 else "mono8"
            )
            
            # Set headers
            ros_time_left = rospy.Time.from_sec(left_ts)
            ros_time_right = rospy.Time.from_sec(right_ts)
            
            left_msg.header.stamp = ros_time_left
            left_msg.header.frame_id = self.config.left_frame_id
            right_msg.header.stamp = ros_time_right
            right_msg.header.frame_id = self.config.right_frame_id
            
            # Publish
            self.left_pub.publish(left_msg)
            self.right_pub.publish(right_msg)
            
        except Exception as e:
            print(f"ROS publishing error: {e}")
    
    def run(self):
        """Main execution loop."""
        # Start receiver thread
        receiver_thread = threading.Thread(target=self.udp_receive_thread, daemon=True)
        receiver_thread.start()
        
        # Setup OpenCV window only if display is enabled
        if self.config.enable_cv_display:
            cv2.namedWindow(self.config.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.config.window_name, self.config.window_width, self.config.window_height)
            cv2.moveWindow(self.config.window_name, self.config.window_x, self.config.window_y)
            print("UDP-ROS receiver started. Press 'q' to quit.")
        else:
            print("UDP-ROS receiver started (no display). Press Ctrl+C to quit.")
        
        try:
            while not rospy.is_shutdown():
                # Get latest stereo pair
                try:
                    stereo_data = self.stereo_queue.get_nowait()
                    self.current_frames.update({
                        'left': stereo_data["left_frame"],
                        'right': stereo_data["right_frame"],
                        'left_ts': stereo_data["left_timestamp"],
                        'right_ts': stereo_data["right_timestamp"]
                    })
                    
                    if not self.first_pair_received:
                        self.first_pair_received = True
                        display_msg = " and display" if self.config.enable_cv_display else ""
                        print(f"First stereo pair received, starting ROS publishing{display_msg}...")
                        
                except Empty:
                    pass
                
                # Process frames
                if self.current_frames['left'] is not None and self.current_frames['right'] is not None:
                    self._process_stereo_pair()
                else:
                    self._handle_no_frames()
                
                # Check for quit (only if display is enabled)
                if self.config.enable_cv_display:
                    if cv2.waitKey(10) & 0xFF == ord('q'):
                        break
                else:
                    # Small delay to prevent excessive CPU usage when no display
                    time.sleep(0.01)
                    
        except KeyboardInterrupt:
            print("\nUser interrupted.")
        finally:
            self._cleanup()
    
    def _process_stereo_pair(self):
        """Process and display stereo pair."""
        try:
            # Apply rotation
            left_rotated = np.rot90(self.current_frames['left'], k=self.config.rotation_k)
            right_rotated = np.rot90(self.current_frames['right'], k=self.config.rotation_k)
            
            # Publish to ROS
            self.publish_to_ros(left_rotated, right_rotated, 
                              self.current_frames['left_ts'], self.current_frames['right_ts'])
            
            # Display only if enabled
            if self.config.enable_cv_display:
                # Convert to BGR for display if needed
                if left_rotated.ndim == 2:
                    left_display = cv2.cvtColor(left_rotated, cv2.COLOR_GRAY2BGR)
                else:
                    left_display = left_rotated
                
                if right_rotated.ndim == 2:
                    right_display = cv2.cvtColor(right_rotated, cv2.COLOR_GRAY2BGR)
                else:
                    right_display = right_rotated
                
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
                    return

                combined_view = np.hstack((left_display, right_display))
                cv2.imshow(self.config.window_name, combined_view)
            
        except Exception as e:
            print(f"Error processing stereo pair: {e}")
    
    def _handle_no_frames(self):
        """Handle case when no frames are available."""
        if self.config.enable_cv_display:
            cv2.imshow(self.config.window_name, self.waiting_img)
    
    def _cleanup(self):
        """Clean shutdown."""
        print("Cleaning up UDP-ROS receiver...")
        self.running.clear()
        
        # Wait for threads
        time.sleep(0.1)  # Give threads time to notice shutdown
        
        # Cleanup resources
        if self.config.enable_cv_display:
            cv2.destroyAllWindows()
        print("UDP-ROS receiver closed.")

def main():
    """Main entry point."""
    rospy.init_node("aria_udp_stereo_receiver", anonymous=True)
    
    # Load configuration from ROS parameters
    config = Config()
    config.listen_ip = rospy.get_param("~listen_ip", config.listen_ip)
    config.listen_port = rospy.get_param("~listen_port", config.listen_port)
    config.rotation_k = rospy.get_param("~rotation_k", config.rotation_k)
    config.verbose_logging = rospy.get_param("~verbose_logging", config.verbose_logging)
    config.enable_cv_display = rospy.get_param("~enable_cv_display", config.enable_cv_display)
    
    # Create and run receiver
    receiver = AriaUDPReceiver(config)
    receiver.run()

if __name__ == "__main__":
    main()
