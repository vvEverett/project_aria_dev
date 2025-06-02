# Aria SLAM UDP Sender
#
# Connects to an Aria device, captures stereo camera images, compresses them using JPEG,
# and streams the compressed data over UDP to a specified target.
# Tracks and prints transmission statistics. Supports Linux iptables update for local forwarding.
#

import sys
import time
import socket
import struct
import numpy as np
import cv2  # Added for JPEG compression
import aria.sdk as aria
from projectaria_tools.core.sensor_data import ImageDataRecord
from common import update_iptables, quit_keypress

# Configuration
DEVICE_IP = "192.168.3.41"
STREAMING_PROFILE = "profile14"
UPDATE_IPTABLES_ON_LINUX = True
FORWARDING_IP = "127.0.0.1"
FORWARDING_PORT = 9999
STATS_PRINT_INTERVAL_SECONDS = 5.0
JPEG_QUALITY = 85  # JPEG compression quality (1-100, higher = better quality but larger size)

class UDPSender:
    """UDP sender for compressed image data with statistics tracking."""
    def __init__(self, host, port):
        self.target_address = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.MAX_UDP_PAYLOAD = 65507 - 28
        
        self.frames_sent_count = 0
        self.bytes_sent_count = 0
        self.compression_ratio_sum = 0.0
        self.start_time = time.time()
        
        print(f"UDP Sender initialized with JPEG compression (quality={JPEG_QUALITY}), target: {host}:{port}")

    def send_frame(self, frame: np.ndarray, camera_id_str: str, timestamp: float):
        try:
            # Convert frame to uint8 if needed
            if frame.dtype != np.uint8:
                frame = frame.astype(np.uint8)
            
            # Get original frame info
            original_height, original_width = frame.shape[:2]
            original_channels = frame.shape[2] if frame.ndim == 3 else 1
            original_size = frame.nbytes
            
            # Convert grayscale to BGR for JPEG encoding if needed
            if frame.ndim == 2:
                frame_for_encoding = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            else:
                frame_for_encoding = frame
            
            # JPEG compress the image
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
            success, compressed_data = cv2.imencode('.jpg', frame_for_encoding, encode_param)
            
            if not success:
                print(f"Failed to compress frame ({camera_id_str})")
                return
            
            compressed_bytes = compressed_data.tobytes()
            compressed_size = len(compressed_bytes)
            
            # Calculate compression ratio
            compression_ratio = original_size / compressed_size if compressed_size > 0 else 1.0
            self.compression_ratio_sum += compression_ratio
            
            cam_id_byte = 1 if camera_id_str == 'left' else 2
            
            # Pack header: timestamp, cam_id, original_height, original_width, original_channels, compressed_size
            header = struct.pack('!dBHHBI', timestamp, cam_id_byte, original_height, original_width, original_channels, compressed_size)
            payload = header + compressed_bytes

            # Check if payload fits in single UDP packet
            if len(payload) > self.MAX_UDP_PAYLOAD:
                print(f"Warning: Compressed frame still too large ({len(payload)} bytes), consider reducing JPEG_QUALITY")
                return
            
            # Send as single packet
            self.sock.sendto(b'COMP' + payload, self.target_address)
            
            self.frames_sent_count += 1
            self.bytes_sent_count += len(payload)

        except Exception as e:
            print(f"Error sending compressed frame ({camera_id_str}): {e}")

    def close(self):
        self.sock.close()
        elapsed_time = time.time() - self.start_time
        avg_fps = self.frames_sent_count / elapsed_time if elapsed_time > 1 else self.frames_sent_count
        avg_mbps = (self.bytes_sent_count * 8 / (1024*1024)) / elapsed_time if elapsed_time > 1 else (self.bytes_sent_count * 8 / (1024*1024))
        avg_compression_ratio = self.compression_ratio_sum / self.frames_sent_count if self.frames_sent_count > 0 else 1.0
        
        print(f"UDP Sender closed.")
        print(f"  Total sent: {self.frames_sent_count} frames, {self.bytes_sent_count / (1024*1024):.2f} MB")
        print(f"  Runtime: {elapsed_time:.2f} seconds")
        print(f"  Average rate: {avg_fps:.1f} FPS, {avg_mbps:.2f} Mbps")
        print(f"  Average compression ratio: {avg_compression_ratio:.1f}x")


class AriaStreamObserver:
    """Observer for caching latest images from Aria SDK."""
    def __init__(self):
        self.latest_data = {}
        self.frames_received_count_left = 0
        self.frames_received_count_right = 0

    def on_image_received(self, image: np.ndarray, record: ImageDataRecord):
        camera_id_str = "left" if record.camera_id == aria.CameraId.Slam1 else "right"
        if camera_id_str == "left":
            self.frames_received_count_left += 1
        else:
            self.frames_received_count_right += 1
            
        self.latest_data[record.camera_id] = {
            "image": image,
            "timestamp": record.capture_timestamp_ns / 1e9
        }

def main():
    if UPDATE_IPTABLES_ON_LINUX and sys.platform.startswith("linux"):
        update_iptables()
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

        print("Attempting to stop any active sessions...")
        streaming_manager = device.streaming_manager
        recording_manager = device.recording_manager
        
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

        streaming_client = streaming_manager.streaming_client
        streaming_config = aria.StreamingConfig()
        streaming_config.profile_name = STREAMING_PROFILE
        streaming_config.security_options.use_ephemeral_certs = True
        streaming_manager.streaming_config = streaming_config
        streaming_manager.start_streaming()
        print(f"Streaming started with profile: {STREAMING_PROFILE}")

        # Configure the streaming client to subscribe to SLAM data
        sub_config = streaming_client.subscription_config
        sub_config.subscriber_data_type = aria.StreamingDataType.Slam
        sub_config.message_queue_size[aria.StreamingDataType.Slam] = 5
        streaming_client.subscription_config = sub_config

        observer = AriaStreamObserver()
        sender = UDPSender(FORWARDING_IP, FORWARDING_PORT)
        streaming_client.set_streaming_client_observer(observer)
        streaming_client.subscribe()
        print("Subscribed to SLAM data stream, starting transmission...")

        last_stats_print_time = time.time()
        frames_since_last_stats_left = 0
        frames_since_last_stats_right = 0
        frames_received_since_last_stats_left = 0
        frames_received_since_last_stats_right = 0
        total_frames_sent_left = 0
        total_frames_sent_right = 0
        last_received_count_left = 0
        last_received_count_right = 0
        
        while not quit_keypress():
            current_loop_time = time.time()
            available_cameras = list(observer.latest_data.keys())
            
            processed_in_loop = False
            for cam_id in available_cameras:
                data = observer.latest_data.pop(cam_id, None)
                if data:
                    processed_in_loop = True
                    camera_id_str = "left" if cam_id == aria.CameraId.Slam1 else "right"
                    original_image = data["image"]
                    sender.send_frame(original_image, camera_id_str, data["timestamp"])
                    
                    if camera_id_str == "left":
                        frames_since_last_stats_left += 1
                        total_frames_sent_left +=1
                    else:
                        frames_since_last_stats_right += 1
                        total_frames_sent_right += 1
            
            if current_loop_time - last_stats_print_time >= STATS_PRINT_INTERVAL_SECONDS:
                elapsed_interval = current_loop_time - last_stats_print_time
                
                # Calculate sent FPS
                fps_sent_left = frames_since_last_stats_left / elapsed_interval if elapsed_interval > 0 else 0
                fps_sent_right = frames_since_last_stats_right / elapsed_interval if elapsed_interval > 0 else 0
                
                # Calculate received FPS
                current_received_left = observer.frames_received_count_left
                current_received_right = observer.frames_received_count_right
                frames_received_since_last_stats_left = current_received_left - last_received_count_left
                frames_received_since_last_stats_right = current_received_right - last_received_count_right
                fps_received_left = frames_received_since_last_stats_left / elapsed_interval if elapsed_interval > 0 else 0
                fps_received_right = frames_received_since_last_stats_right / elapsed_interval if elapsed_interval > 0 else 0
                
                print(f"STATUS [{time.strftime('%H:%M:%S')}]: "
                      f"L_Recv={fps_received_left:.1f}, L_Send={fps_sent_left:.1f} | "
                      f"R_Recv={fps_received_right:.1f}, R_Send={fps_sent_right:.1f} | "
                      f"Total L={total_frames_sent_left}, R={total_frames_sent_right}")
                
                frames_since_last_stats_left = 0
                frames_since_last_stats_right = 0
                last_received_count_left = current_received_left
                last_received_count_right = current_received_right
                last_stats_print_time = current_loop_time

            if not processed_in_loop:
                time.sleep(0.005)
            else:
                time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nUser interrupted (Ctrl+C).")
    except Exception as e:
        print(f"Error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("=" * 50)
        print("Shutting down, cleaning up resources...")
        
        if sender is not None:
            print("  Closing UDP sender...")
            sender.close()
        else:
            print("  UDP sender was not initialized.")

        if device is not None:
            if streaming_client is not None:
                try:
                    print("  Unsubscribing from Aria stream...")
                    streaming_client.unsubscribe()
                    print("    Successfully unsubscribed.")
                except Exception as e_unsub:
                    print(f"    Error unsubscribing: {e_unsub}")
            
            if streaming_manager is not None:
                try:
                    print(f"  Stopping Aria device streaming...")
                    streaming_manager.stop_streaming()
                    print("    Streaming stopped successfully.")
                except Exception as e_sm:
                    print(f"    Error stopping stream: {e_sm}")
            
            try:
                print(f"  Disconnecting from Aria device...")
                device_client.disconnect(device)
                print("    Disconnected from device.")
            except Exception as e_dc:
                print(f"    Error disconnecting: {e_dc}")
        else:
            print("  Aria device was not connected.")
        
        time.sleep(0.5)
        print("Cleanup complete.")
        print("=" * 50)

if __name__ == "__main__":
    main()