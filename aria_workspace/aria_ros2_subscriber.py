#!/usr/bin/env python3
"""
Aria Simple ROS 2 Subscriber - All images in one window

Displays all 6 camera feeds in a single window arranged in a 2x3 grid.
"""

import sys
import time
import threading

# ROS 2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
except ImportError:
    print("Error: ROS 2 dependencies not installed. Please install rclpy, sensor_msgs, cv_bridge")
    sys.exit(1)

# OpenCV import
try:
    import cv2
    import numpy as np
except ImportError:
    print("Error: OpenCV not installed. Please install python3-opencv")
    sys.exit(1)

# Same QoS profile as publisher
IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE
)

class AriaSimpleSubscriber(Node):
    """Subscriber that displays all images in one window."""
    
    def __init__(self):
        super().__init__('aria_simple_subscriber')
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Statistics tracking
        self.message_count = {
            'slam_left_raw': 0,
            'slam_left_undistorted': 0,
            'slam_right_raw': 0,
            'slam_right_undistorted': 0,
            'rgb_raw': 0,
            'rgb_undistorted': 0
        }
        
        self.last_message_time = {topic: 0 for topic in self.message_count.keys()}
        self.start_time = time.time()
        self.last_stats_time = self.start_time
        
        # Image storage for display
        self.current_images = {topic: None for topic in self.message_count.keys()}
        self.image_lock = threading.Lock()
        
        # Create subscribers
        topics = {
            'slam_left_raw': '/aria/slam_left/raw',
            'slam_left_undistorted': '/aria/slam_left/undistorted',
            'slam_right_raw': '/aria/slam_right/raw',
            'slam_right_undistorted': '/aria/slam_right/undistorted',
            'rgb_raw': '/aria/rgb/raw',
            'rgb_undistorted': '/aria/rgb/undistorted'
        }
        
        self.subscribers = {}
        for key, topic in topics.items():
            self.subscribers[key] = self.create_subscription(
                Image, topic, 
                lambda msg, k=key: self.image_callback(msg, k), 
                IMAGE_QOS
            )
        
        # Timer for statistics and display
        self.stats_timer = self.create_timer(2.0, self.print_statistics)
        self.display_timer = self.create_timer(0.033, self.update_display)  # ~30 FPS display
        
        # OpenCV window
        cv2.namedWindow('Aria Camera Feeds', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Aria Camera Feeds', 1000, 800)
        
        self.get_logger().info("Aria Subscriber initialized - All images in one window")
    
    def image_callback(self, msg, topic_key):
        """Process incoming image messages."""
        current_time = time.time()
        self.message_count[topic_key] += 1
        self.last_message_time[topic_key] = current_time
        
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Store image for display
            with self.image_lock:
                self.current_images[topic_key] = cv_image
                
        except Exception as e:
            self.get_logger().error(f"Error processing {topic_key}: {e}")
    
    def create_combined_display(self):
        """Create a combined display with all camera feeds maintaining original aspect ratios."""
        with self.image_lock:
            images = self.current_images.copy()
        
        # Define display order (2 rows x 3 columns)
        display_order = [
            ['slam_left_raw', 'slam_right_raw', 'rgb_raw'],
            ['slam_left_undistorted', 'slam_right_undistorted', 'rgb_undistorted']
        ]
        
        # Define target dimensions based on actual camera specifications
        # SLAM cameras: 480x640 (3:4 aspect ratio - portrait orientation)
        slam_target_height = 320
        slam_target_width = int(slam_target_height * (480/640))  # 240 pixels
        
        # RGB cameras: 512x512 (1:1 aspect ratio)
        rgb_target_height = 320
        rgb_target_width = rgb_target_height  # 320 pixels (square)
        
        rows = []
        for row_topics in display_order:
            row_images = []
            for topic in row_topics:
                img = images.get(topic)
                
                # Determine target size based on camera type
                if 'slam' in topic:
                    target_width = slam_target_width
                    target_height = slam_target_height
                else:  # RGB
                    target_width = rgb_target_width
                    target_height = rgb_target_height
                
                if img is not None:
                    # Resize image to exact target dimensions
                    resized = cv2.resize(img, (target_width, target_height))
                    
                    # Add topic label
                    cv2.putText(resized, topic, (10, 30), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                else:
                    # Create placeholder with appropriate size
                    canvas = np.zeros((target_height, target_width, 3), dtype=np.uint8)
                    cv2.putText(canvas, f"{topic} - No Image", (10, target_height//2), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    resized = canvas
                
                row_images.append(resized)
            
            # Combine images in this row
            if row_images:
                rows.append(np.hstack(row_images))
        
        # Combine all rows
        if rows:
            combined = np.vstack(rows)
            return combined
        else:
            # Return placeholder if no images
            total_width = slam_target_width * 2 + rgb_target_width
            placeholder = np.zeros((600, total_width, 3), dtype=np.uint8)
            cv2.putText(placeholder, "Waiting for images...", (total_width//2 - 150, 300), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            return placeholder
    
    def update_display(self):
        """Update the combined display window."""
        try:
            combined_image = self.create_combined_display()
            cv2.imshow('Aria Camera Feeds', combined_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error updating display: {e}")
    
    def print_statistics(self):
        """Print subscription statistics."""
        current_time = time.time()
        time_diff = current_time - self.last_stats_time
        total_runtime = current_time - self.start_time
        
        if time_diff < 0.1:  # Avoid division by zero
            return
            
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"Subscriber Stats (Runtime: {total_runtime:.1f}s)")
        self.get_logger().info("=" * 70)
        
        # Calculate and display FPS for each topic
        for topic_name, count in self.message_count.items():
            fps = count / time_diff if time_diff > 0 else 0
            total_fps = count / total_runtime if total_runtime > 0 else 0
            last_msg_ago = current_time - self.last_message_time[topic_name] if self.last_message_time[topic_name] > 0 else float('inf')
            
            status = "ACTIVE" if last_msg_ago < 1.0 else "INACTIVE"
            
            self.get_logger().info(
                f"{topic_name:22} | {status:8} | FPS: {fps:5.1f} | Avg: {total_fps:5.1f} | Count: {count:5}"
            )
        
        self.get_logger().info("=" * 70)
        
        # Reset interval counters
        for topic_name in self.message_count:
            self.message_count[topic_name] = 0
        self.last_stats_time = current_time


def main():
    """Main function."""
    print("Aria ROS 2 Subscriber - All Images in One Window")
    print("Displays all 6 camera feeds in a single window (2x3 grid)")
    print("Press Ctrl+C to stop or 'q' in the image window")
    
    rclpy.init()
    
    subscriber = AriaSimpleSubscriber()
    
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        print("\nSubscriber interrupted by user")
    except Exception as e:
        print(f"Error in subscriber: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Cleaning up subscriber...")
        cv2.destroyAllWindows()
        subscriber.destroy_node()
        rclpy.shutdown()
        print("Subscriber shutdown complete.")


if __name__ == '__main__':
    main()