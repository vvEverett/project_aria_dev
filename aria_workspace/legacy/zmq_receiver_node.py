#!/usr/bin/env python3
# Aria SLAM ZMQ Stereo Receiver (ROS1 Node)
#
# Receives stereo camera streams via ZeroMQ (ZMQ) IPC and publishes them as ROS1 sensor_msgs/Image topics.
# Designed to run outside the Docker container and connect to a containerized sender.
# Publishes: /aria/left/image_raw, /aria/right/image_raw (sensor_msgs/Image)
# Press 'q' to quit the node.

import struct
import numpy as np
import cv2
import time
import threading
from queue import LifoQueue, Empty
from dataclasses import dataclass
from typing import Optional, Dict, Any

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    import zmq
except ImportError:
    print("Error: pyzmq not installed. Install with: pip install pyzmq")
    exit(1)

@dataclass
class Config:
    """Centralized configuration class."""
    # ZMQ settings
    zmq_connect_address: str = "ipc:///tmp/aria_slam.ipc"
    zmq_recv_timeout: int = 1000  # milliseconds
    
    # Image processing
    rotation_k: int = -1  # np.rot90 k parameter
    
    # Connection monitoring
    heartbeat_timeout: float = 5.0
    verbose_logging: bool = False
    
    # Performance settings
    queue_maxsize: int = 1
    stats_interval: float = 5.0
    
    # Display settings
    enable_cv_display: bool = False  # Control OpenCV window display
    window_name: str = "Aria SLAM Stream (ZMQ-ROS)"
    window_width: int = 960
    window_height: int = 640
    window_x: int = 50
    window_y: int = 50
    
    # ROS settings
    ros_queue_size: int = 10
    left_topic: str = "/aria/left/image_raw"
    right_topic: str = "/aria/right/image_raw"
    rgb_topic: str = "/aria/rgb/image_raw"
    rgb_gray_topic: str = "/aria/rgb/image_gray"
    left_frame_id: str = "aria_left"
    right_frame_id: str = "aria_right"
    rgb_frame_id: str = "aria_rgb"

class FrameData:
    """Efficient frame data container."""
    __slots__ = ['id', 'frame', 'timestamp']
    
    def __init__(self, camera_id: str, frame: np.ndarray, timestamp: float):
        self.id = camera_id
        self.frame = frame
        self.timestamp = timestamp

class AriaZMQReceiver:
    """High-performance Aria ZMQ to ROS converter."""
    
    def __init__(self, config: Config):
        self.config = config
        self.context = zmq.Context()
        self.stereo_queue = LifoQueue(maxsize=config.queue_maxsize)
        self.rgb_queue = LifoQueue(maxsize=config.queue_maxsize)  # Separate RGB queue
        self.running = threading.Event()
        self.connection_status = {'connected': False, 'last_heartbeat': 0, 'last_frame': 0}
        
        # ROS components
        self.bridge = CvBridge()
        self.left_pub = rospy.Publisher(config.left_topic, Image, queue_size=config.ros_queue_size)
        self.right_pub = rospy.Publisher(config.right_topic, Image, queue_size=config.ros_queue_size)
        self.rgb_pub = rospy.Publisher(config.rgb_topic, Image, queue_size=config.ros_queue_size)
        self.rgb_gray_pub = rospy.Publisher(config.rgb_gray_topic, Image, queue_size=config.ros_queue_size)
        
        # State tracking
        self.first_pair_received = False
        self.first_rgb_received = False
        self.connection_warning_shown = False
        self.current_frames = {'left': None, 'right': None, 'left_ts': 0, 'right_ts': 0}
        self.current_rgb = {'frame': None, 'timestamp': 0}
        
        # Pre-allocate commonly used objects
        self.header_format = '!dBHHB'
        self.header_size = struct.calcsize(self.header_format)
        self.waiting_img = np.zeros((480, 640, 3), dtype=np.uint8)
    
    def parse_frame_payload(self, payload: bytes) -> Optional[FrameData]:
        """Optimized frame payload parser."""
        if len(payload) < self.header_size:
            return None
        
        try:
            timestamp, cam_id_byte, height, width, channels = struct.unpack(
                self.header_format, payload[:self.header_size]
            )
            
            frame_data = payload[self.header_size:]
            expected_size = height * width * channels
            
            if len(frame_data) != expected_size:
                if self.config.verbose_logging:
                    print(f"Incomplete data! Expected:{expected_size}, Actual:{len(frame_data)}")
                return None
            
            # Efficient frame reconstruction
            shape = (height, width, channels) if channels > 1 else (height, width)
            frame = np.frombuffer(frame_data, dtype=np.uint8).reshape(shape)
            
            # Updated camera ID mapping to handle RGB
            if cam_id_byte == 1:
                camera_id = "left"
            elif cam_id_byte == 2:
                camera_id = "right"
            elif cam_id_byte == 3:
                camera_id = "rgb"
            else:
                camera_id = f"unknown_{cam_id_byte}"
            
            return FrameData(camera_id, frame, timestamp)
            
        except Exception as e:
            if self.config.verbose_logging:
                print(f"Error parsing payload: {e}")
            return None
    
    def zmq_receive_thread(self):
        """Optimized ZMQ receiver thread."""
        socket = self.context.socket(zmq.SUB)
        socket.connect(self.config.zmq_connect_address)
        
        # Subscribe to topics including RGB
        for topic in [b"slam.left", b"slam.right", b"slam.rgb", b"heartbeat"]:
            socket.setsockopt(zmq.SUBSCRIBE, topic)
        
        socket.setsockopt(zmq.RCVTIMEO, self.config.zmq_recv_timeout)
        
        print(f"ZMQ receiver connected to: {self.config.zmq_connect_address}")
        print("Subscribed to: slam.left, slam.right, slam.rgb, heartbeat")
        
        # Efficient state tracking
        temp_buffer = {}
        frame_counts = {"left": 0, "right": 0, "rgb": 0}
        last_stats_time = time.time()
        last_frame_counts = {"left": 0, "right": 0, "rgb": 0}
        last_heartbeat_time = time.time()
        
        self.running.set()
        
        while self.running.is_set() and not rospy.is_shutdown():
            try:
                topic, payload = socket.recv_multipart()
                topic_str = topic.decode('utf-8')
                current_time = time.time()
                
                if topic_str == "heartbeat":
                    if len(payload) >= 8:
                        last_heartbeat_time = current_time
                        self.connection_status.update({
                            'last_heartbeat': current_time,
                            'connected': True
                        })
                
                elif topic_str.startswith("slam."):
                    frame_data = self.parse_frame_payload(payload)
                    if frame_data:
                        camera_id = frame_data.id
                        frame_counts[camera_id] += 1
                        
                        self.connection_status.update({
                            'last_frame': current_time,
                            'connected': True
                        })
                        
                        # Handle RGB separately from stereo
                        if camera_id == "rgb":
                            rgb_data = {
                                "frame": frame_data.frame,
                                "timestamp": frame_data.timestamp
                            }
                            try:
                                self.rgb_queue.put_nowait(rgb_data)
                            except:
                                try:
                                    self.rgb_queue.get_nowait()
                                    self.rgb_queue.put_nowait(rgb_data)
                                except:
                                    pass
                        else:
                            # Handle stereo cameras
                            temp_buffer[camera_id] = frame_data
                        
                        # Stats reporting
                        if current_time - last_stats_time > self.config.stats_interval:
                            self._print_rgb_stats(frame_counts, last_frame_counts, 
                                                current_time - last_stats_time, current_time)
                            last_frame_counts = frame_counts.copy()
                            last_stats_time = current_time
                        
                        # Stereo pair assembly (only for left/right)
                        if len(temp_buffer) == 2 and "left" in temp_buffer and "right" in temp_buffer:
                            stereo_data = {
                                "left_frame": temp_buffer["left"].frame,
                                "right_frame": temp_buffer["right"].frame,
                                "left_timestamp": temp_buffer["left"].timestamp,
                                "right_timestamp": temp_buffer["right"].timestamp
                            }
                            
                            try:
                                self.stereo_queue.put_nowait(stereo_data)
                            except:
                                try:
                                    self.stereo_queue.get_nowait()
                                    self.stereo_queue.put_nowait(stereo_data)
                                except:
                                    pass
                            
                            temp_buffer.clear()
                
                # Connection timeout check
                if current_time - last_heartbeat_time > self.config.heartbeat_timeout:
                    if self.connection_status['connected']:
                        print(f"Warning: No heartbeat for {self.config.heartbeat_timeout}s")
                        self.connection_status['connected'] = False
                        
            except zmq.Again:
                # Timeout - check connection
                if time.time() - last_heartbeat_time > self.config.heartbeat_timeout:
                    if self.connection_status['connected']:
                        print("Connection timeout - waiting for sender...")
                        self.connection_status['connected'] = False
                continue
                
            except Exception as e:
                if self.running.is_set():
                    print(f"ZMQ receiver error: {e}")
        
        socket.close()
        print("ZMQ receiver thread stopped.")
    
    def _print_stats(self, frame_counts: Dict[str, int], last_counts: Dict[str, int], 
                    time_diff: float, current_time: float):
        """Print performance statistics."""
        left_fps = (frame_counts["left"] - last_counts["left"]) / time_diff
        right_fps = (frame_counts["right"] - last_counts["right"]) / time_diff
        timestamp = time.strftime("%H:%M:%S", time.localtime(current_time))
        
        print(f"[{timestamp}] FPS: L={left_fps:.1f}, R={right_fps:.1f} | "
              f"Total: L={frame_counts['left']}, R={frame_counts['right']}")
    
    def _print_rgb_stats(self, frame_counts: Dict[str, int], last_counts: Dict[str, int], 
                        time_diff: float, current_time: float):
        """Print performance statistics including RGB."""
        left_fps = (frame_counts["left"] - last_counts["left"]) / time_diff
        right_fps = (frame_counts["right"] - last_counts["right"]) / time_diff
        rgb_fps = (frame_counts["rgb"] - last_counts["rgb"]) / time_diff
        timestamp = time.strftime("%H:%M:%S", time.localtime(current_time))
        
        print(f"[{timestamp}] FPS: L={left_fps:.1f}, R={right_fps:.1f}, RGB={rgb_fps:.1f} | "
              f"Total: L={frame_counts['left']}, R={frame_counts['right']}, RGB={frame_counts['rgb']}")
    
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
    
    def _process_rgb_frame(self):
        """Process and publish RGB frame."""
        try:
            # Apply rotation
            rgb_rotated = np.rot90(self.current_rgb['frame'], k=self.config.rotation_k)
            
            # Publish to ROS (both original and grayscale)
            self.publish_rgb_to_ros(rgb_rotated, self.current_rgb['timestamp'])
            
        except Exception as e:
            print(f"Error processing RGB frame: {e}")

    def publish_rgb_to_ros(self, rgb_frame: np.ndarray, timestamp: float):
        """Efficient RGB publishing - both original and grayscale."""
        try:
            ros_time = rospy.Time.from_sec(timestamp)
            
            # Determine if input is grayscale or color
            if rgb_frame.ndim == 2 or (rgb_frame.ndim == 3 and rgb_frame.shape[2] == 1):
                # Input is grayscale - publish as mono8 for gray topic
                gray_msg = self.bridge.cv2_to_imgmsg(rgb_frame.squeeze(), encoding="mono8")
                gray_msg.header.stamp = ros_time
                gray_msg.header.frame_id = self.config.rgb_frame_id
                self.rgb_gray_pub.publish(gray_msg)
                
                # For RGB topic, convert grayscale to BGR for consistency
                if rgb_frame.ndim == 2:
                    rgb_bgr = cv2.cvtColor(rgb_frame, cv2.COLOR_GRAY2BGR)
                else:
                    rgb_bgr = cv2.cvtColor(rgb_frame.squeeze(), cv2.COLOR_GRAY2BGR)
                    
                rgb_msg = self.bridge.cv2_to_imgmsg(rgb_bgr, encoding="bgr8")
                
            else:
                # Input is color - publish original as RGB and convert to grayscale
                rgb_msg = self.bridge.cv2_to_imgmsg(rgb_frame, encoding="bgr8")
                
                # Convert to grayscale for gray topic
                gray_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2GRAY)
                gray_msg = self.bridge.cv2_to_imgmsg(gray_frame, encoding="mono8")
                gray_msg.header.stamp = ros_time
                gray_msg.header.frame_id = self.config.rgb_frame_id
                self.rgb_gray_pub.publish(gray_msg)
            
            rgb_msg.header.stamp = ros_time
            rgb_msg.header.frame_id = self.config.rgb_frame_id
            self.rgb_pub.publish(rgb_msg)
            
        except Exception as e:
            print(f"RGB ROS publishing error: {e}")
    
    def run(self):
        """Main execution loop."""
        # Start receiver thread
        receiver_thread = threading.Thread(target=self.zmq_receive_thread, daemon=True)
        receiver_thread.start()
        
        # Setup OpenCV window only if display is enabled
        if self.config.enable_cv_display:
            cv2.namedWindow(self.config.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.config.window_name, self.config.window_width, self.config.window_height)
            cv2.moveWindow(self.config.window_name, self.config.window_x, self.config.window_y)
            print("ZMQ-ROS receiver started. Press 'q' to quit.")
        else:
            print("ZMQ-ROS receiver started (no display). Press Ctrl+C to quit.")
        
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
                
                # Get latest RGB frame
                try:
                    rgb_data = self.rgb_queue.get_nowait()
                    self.current_rgb.update({
                        'frame': rgb_data["frame"],
                        'timestamp': rgb_data["timestamp"]
                    })
                    
                    if not self.first_rgb_received:
                        self.first_rgb_received = True
                        print("First RGB frame received, starting RGB publishing...")
                        
                except Empty:
                    pass
                
                # Process frames
                if self.current_frames['left'] is not None and self.current_frames['right'] is not None:
                    self._process_stereo_pair()
                else:
                    self._handle_no_frames()
                
                # Process RGB frame
                if self.current_rgb['frame'] is not None:
                    self._process_rgb_frame()
                
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
                stereo_display = np.hstack([left_rotated, right_rotated])
                if not stereo_display.flags['C_CONTIGUOUS']:
                    stereo_display = np.ascontiguousarray(stereo_display)
                
                cv2.imshow(self.config.window_name, stereo_display)
            
        except Exception as e:
            print(f"Error processing stereo pair: {e}")
    
    def _handle_no_frames(self):
        """Handle case when no frames are available."""
        if self.config.enable_cv_display:
            cv2.imshow(self.config.window_name, self.waiting_img)
        
        if not self.connection_status['connected'] and not self.connection_warning_shown:
            print("Waiting for connection to ZMQ sender...")
            self.connection_warning_shown = True
        elif self.connection_status['connected'] and self.connection_warning_shown:
            print("Connected! Waiting for frames...")
            self.connection_warning_shown = False
    
    def _cleanup(self):
        """Clean shutdown."""
        print("Cleaning up ZMQ-ROS receiver...")
        self.running.clear()
        
        # Wait for threads
        time.sleep(0.1)  # Give threads time to notice shutdown
        
        # Cleanup resources
        self.context.term()
        if self.config.enable_cv_display:
            cv2.destroyAllWindows()
        print("ZMQ-ROS receiver closed.")

def main():
    """Main entry point."""
    rospy.init_node("aria_zmq_stereo_receiver", anonymous=True)
    
    # Load configuration from ROS parameters
    config = Config()
    config.zmq_connect_address = rospy.get_param("~zmq_connect_address", config.zmq_connect_address)
    config.rotation_k = rospy.get_param("~rotation_k", config.rotation_k)
    config.verbose_logging = rospy.get_param("~verbose_logging", config.verbose_logging)
    config.heartbeat_timeout = rospy.get_param("~heartbeat_timeout", config.heartbeat_timeout)
    config.enable_cv_display = rospy.get_param("~enable_cv_display", config.enable_cv_display)
    
    # Create and run receiver
    receiver = AriaZMQReceiver(config)
    receiver.run()

if __name__ == "__main__":
    main()
