# Aria Camera Timestamp Sync Analysis - USB Version
#
# Connects to an Aria device via USB and analyzes timestamp synchronization
# between SLAM and RGB cameras without sending data via ZMQ.
#
# Prerequisites:
# - Aria SDK (projectaria_client_sdk)

import sys
import time
import threading
import numpy as np
try:
    import aria.sdk as aria
    from projectaria_tools.core.sensor_data import ImageDataRecord
except ImportError:
    print("Error: Project Aria SDK not installed. Please install projectaria_client_sdk")
    sys.exit(1)
    
from common import quit_keypress, ctrl_c_handler
from aria_utils import (
    CalibrationManager, BaseAriaStreamObserver, 
    setup_aria_sdk, get_camera_id_string
)

# Configuration
STREAMING_PROFILE = "profile12"  # USB streaming profile
ENABLE_RGB_STREAM = True  # Set to True to enable RGB stream for sync analysis
TIMESTAMP_REPORT_INTERVAL = 5.0  # Report interval in seconds

class AriaUSBDeviceManager:
    """Manages Aria device USB connection and streaming configuration."""
    
    def __init__(self, streaming_profile, enable_rgb_stream=False):
        self.streaming_profile = streaming_profile
        self.enable_rgb_stream = enable_rgb_stream
        self.device_client = aria.DeviceClient()
        self.device = None
        self.streaming_manager = None
        self.recording_manager = None
        self.streaming_client = None
        
    def connect(self):
        """Connect to Aria device via USB and initialize managers."""
        print("Connecting to Aria device via USB...")
        
        self.device = self.device_client.connect()
        print("Device connected successfully via USB.")
        
        info = self.device.info
        status = self.device.status
        print(f"Device info - Model: {info.model}, Serial: {info.serial}")
        print(f"Device status - Battery: {status.battery_level}%, Mode: {status.device_mode}")
        
        self.streaming_manager = self.device.streaming_manager
        self.recording_manager = self.device.recording_manager
        
    def stop_existing_sessions(self):
        """Stop any existing streaming or recording sessions."""
        print("Stopping any existing sessions...")
        
        try:
            self.streaming_manager.stop_streaming()
            print("  Existing streaming stopped.")
        except Exception as e:
            print(f"  Note (stopping stream): {e}")
            
        try:
            self.recording_manager.stop_recording()
            print("  Existing recording stopped.")
        except Exception as e:
            print(f"  Note (stopping recording): {e}")
        
        time.sleep(3)
        print("Pre-start cleanup finished.\n")
    
    def start_streaming(self):
        """Configure and start USB streaming with SLAM data subscription and optional RGB."""
        self.streaming_client = self.streaming_manager.streaming_client
        
        streaming_config = aria.StreamingConfig()
        streaming_config.profile_name = self.streaming_profile
        streaming_config.streaming_interface = aria.StreamingInterface.Usb
        streaming_config.security_options.use_ephemeral_certs = True
        self.streaming_manager.streaming_config = streaming_config
        
        try:
            self.streaming_manager.start_streaming()
            print(f"USB streaming started with profile: {self.streaming_profile}")
            
            time.sleep(2)
            state = self.streaming_manager.streaming_state
            print(f"Current streaming state: {state}")
            
        except Exception as e:
            print(f"Error starting USB streaming: {e}")
            raise

        sub_config = self.streaming_client.subscription_config
        sub_config.subscriber_data_type = aria.StreamingDataType.Slam
        sub_config.message_queue_size[aria.StreamingDataType.Slam] = 5
        
        if self.enable_rgb_stream:
            sub_config.subscriber_data_type = sub_config.subscriber_data_type | aria.StreamingDataType.Rgb
            sub_config.message_queue_size[aria.StreamingDataType.Rgb] = 5
            print("RGB stream enabled for sync analysis")
        
        self.streaming_client.subscription_config = sub_config
    
    def cleanup(self):
        """Clean shutdown of device connections."""
        if self.device is not None:
            try:
                if self.streaming_client is not None:
                    print("  Unsubscribing from Aria stream...")
                    self.streaming_client.unsubscribe()
                    print("    Successfully unsubscribed.")
            except Exception as e:
                print(f"    Error unsubscribing: {e}")
            
            try:
                if self.streaming_manager is not None:
                    print("  Stopping Aria device streaming...")
                    self.streaming_manager.stop_streaming()
                    print("    Streaming stopped successfully.")
            except Exception as e:
                print(f"    Error stopping stream: {e}")
            
            try:
                print("  Disconnecting from Aria device...")
                self.device_client.disconnect(self.device)
                print("    Disconnected from device.")
            except Exception as e:
                print(f"    Error disconnecting: {e}")
        else:
            print("  Aria device was not connected.")


class TimestampSyncAnalyzer(BaseAriaStreamObserver):
    """Analyzes timestamp synchronization between different cameras."""
    
    def __init__(self):
        super().__init__(None)  # No calibration needed for timestamp analysis
        self.data_lock = threading.Lock()
        
        # Timestamp tracking for each camera
        self.last_timestamps = {}
        self.frame_counts = {}
        
        # Synchronization tracking
        self.sync_stats = {
            'left_rgb_diffs': [],
            'right_rgb_diffs': [],
            'left_right_diffs': [],
            'sync_anomaly_count': 0
        }
        
        # Initialize for expected cameras
        expected_cameras = [aria.CameraId.Slam1, aria.CameraId.Slam2]
        if ENABLE_RGB_STREAM:
            expected_cameras.append(aria.CameraId.Rgb)
            
        for camera_id in expected_cameras:
            self.frame_counts[camera_id] = 0
    
    def on_image_received(self, image: np.ndarray, record: ImageDataRecord):
        """Called when a new image is received from any camera."""
        camera_id = record.camera_id
        
        with self.data_lock:
            # Update frame count before analysis
            self.frame_counts[camera_id] = self.frame_counts.get(camera_id, 0) + 1
            
            # Output detailed frame info every 10 frames for each camera
            if self.frame_counts[camera_id] % 10 == 0:
                camera_name = get_camera_id_string(camera_id)
                timestamp_sec = record.capture_timestamp_ns / 1e9
                timestamp_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(timestamp_sec))
                timestamp_ms = (timestamp_sec - int(timestamp_sec)) * 1000
                
                print(f"\n--- Frame {self.frame_counts[camera_id]} from {camera_name} ---")
                print(f"Timestamp: {timestamp_str}.{int(timestamp_ms):03d} ({record.capture_timestamp_ns} ns)")
                print(f"Camera ID: {camera_id}")
                print(f"Resolution: {image.shape[1]}x{image.shape[0]}")
                
                if hasattr(record, 'temperature_c') and record.temperature_c is not None:
                    print(f"Sensor temperature: {record.temperature_c:.1f}°C")
                    
                print("-------------------------------")
        
        self._analyze_timestamp_sync(record)
    
    def _analyze_timestamp_sync(self, record):
        """Analyze timestamp synchronization between cameras."""
        camera_id = record.camera_id
        current_timestamp = record.capture_timestamp_ns / 1e9  # Convert to seconds
        
        with self.data_lock:
            # Check sync with other cameras
            self._check_camera_sync(camera_id, current_timestamp)
            
            # Update last timestamp
            self.last_timestamps[camera_id] = current_timestamp
    
    def _check_camera_sync(self, current_camera_id, current_timestamp):
        """Check synchronization between different cameras."""
        if not ENABLE_RGB_STREAM:
            return
            
        max_time_gap = 0.05  # 50ms window to find corresponding frames
        sync_warning_threshold = 10.0  # Warn if sync difference > 10ms
        
        cameras_to_check = []
        
        if current_camera_id == aria.CameraId.Slam1:
            if aria.CameraId.Slam2 in self.last_timestamps:
                cameras_to_check.append((aria.CameraId.Slam2, 'Left vs Right SLAM', 'left_right_diffs'))
            if aria.CameraId.Rgb in self.last_timestamps:
                cameras_to_check.append((aria.CameraId.Rgb, 'Left SLAM vs RGB', 'left_rgb_diffs'))
                
        elif current_camera_id == aria.CameraId.Slam2:
            if aria.CameraId.Rgb in self.last_timestamps:
                cameras_to_check.append((aria.CameraId.Rgb, 'Right SLAM vs RGB', 'right_rgb_diffs'))
                
        elif current_camera_id == aria.CameraId.Rgb:
            if aria.CameraId.Slam1 in self.last_timestamps:
                cameras_to_check.append((aria.CameraId.Slam1, 'RGB vs Left SLAM', 'left_rgb_diffs'))
            if aria.CameraId.Slam2 in self.last_timestamps:
                cameras_to_check.append((aria.CameraId.Slam2, 'RGB vs Right SLAM', 'right_rgb_diffs'))
        
        for other_camera_id, pair_name, stats_key in cameras_to_check:
            other_timestamp = self.last_timestamps[other_camera_id]
            time_gap = abs(current_timestamp - other_timestamp)
            
            if time_gap <= max_time_gap:
                sync_diff_ms = time_gap * 1000
                self.sync_stats[stats_key].append(sync_diff_ms)
                
                if sync_diff_ms > sync_warning_threshold:
                    self.sync_stats['sync_anomaly_count'] += 1
                    current_camera_name = get_camera_id_string(current_camera_id)
                    other_camera_name = get_camera_id_string(other_camera_id)
                    print(f"SYNC WARNING: {pair_name} timestamp difference: {sync_diff_ms:.2f}ms")
                    print(f"  {current_camera_name}: {current_timestamp:.6f}s")
                    print(f"  {other_camera_name}: {other_timestamp:.6f}s")
        
        # Keep only recent measurements
        for key in ['left_rgb_diffs', 'right_rgb_diffs', 'left_right_diffs']:
            if len(self.sync_stats[key]) > 100:
                self.sync_stats[key] = self.sync_stats[key][-100:]
    
    def get_sync_report(self):
        """Generate synchronization analysis report."""
        with self.data_lock:
            report = {}
            
            # Frame counts
            report['frame_counts'] = {}
            for camera_id, count in self.frame_counts.items():
                camera_name = get_camera_id_string(camera_id)
                report['frame_counts'][camera_name] = count
            
            # Sync statistics
            report['sync_stats'] = {}
            for pair_name, diffs in [
                ('Left SLAM vs RGB', self.sync_stats['left_rgb_diffs']),
                ('Right SLAM vs RGB', self.sync_stats['right_rgb_diffs']),
                ('Left vs Right SLAM', self.sync_stats['left_right_diffs'])
            ]:
                if diffs:
                    avg_diff = sum(diffs) / len(diffs)
                    max_diff = max(diffs)
                    min_diff = min(diffs)
                    
                    report['sync_stats'][pair_name] = {
                        'avg_diff_ms': avg_diff,
                        'max_diff_ms': max_diff,
                        'min_diff_ms': min_diff,
                        'sample_count': len(diffs),
                        'within_1ms_percent': len([d for d in diffs if d <= 1.0]) / len(diffs) * 100,
                        'within_5ms_percent': len([d for d in diffs if d <= 5.0]) / len(diffs) * 100,
                        'within_10ms_percent': len([d for d in diffs if d <= 10.0]) / len(diffs) * 100
                    }
            
            report['total_anomalies'] = self.sync_stats['sync_anomaly_count']
            
            return report


def main():
    """Main execution function for timestamp sync analysis."""
    
    setup_aria_sdk()
    
    device_manager = AriaUSBDeviceManager(STREAMING_PROFILE, ENABLE_RGB_STREAM)
    analyzer = None

    try:
        device_manager.connect()
        device_manager.stop_existing_sessions()

        analyzer = TimestampSyncAnalyzer()

        device_manager.start_streaming()
        device_manager.streaming_client.set_streaming_client_observer(analyzer)
        device_manager.streaming_client.subscribe()
        
        stream_types = "SLAM"
        if ENABLE_RGB_STREAM:
            stream_types += " + RGB"
        print(f"Starting timestamp synchronization analysis for {stream_types} cameras...")

        last_report_time = time.time()
        
        with ctrl_c_handler() as ctrl_c:
            while not (quit_keypress() or ctrl_c):
                current_time = time.time()
                
                # Print sync report periodically
                if current_time - last_report_time > TIMESTAMP_REPORT_INTERVAL:
                    report = analyzer.get_sync_report()
                    
                    print("\n" + "="*60)
                    print("CAMERA TIMESTAMP SYNCHRONIZATION ANALYSIS")
                    print("="*60)
                    
                    # Frame counts
                    print("Frame Counts:")
                    for camera_name, count in report['frame_counts'].items():
                        print(f"  {camera_name}: {count} frames")
                    print()
                    
                    # Sync statistics
                    if report['sync_stats']:
                        print("Synchronization Analysis:")
                        for pair_name, stats in report['sync_stats'].items():
                            print(f"{pair_name}:")
                            print(f"  Average difference: {stats['avg_diff_ms']:.2f}ms")
                            print(f"  Range: {stats['min_diff_ms']:.2f} - {stats['max_diff_ms']:.2f}ms")
                            print(f"  Samples: {stats['sample_count']}")
                            print(f"  Within 1ms: {stats['within_1ms_percent']:.1f}% | "
                                  f"5ms: {stats['within_5ms_percent']:.1f}% | "
                                  f"10ms: {stats['within_10ms_percent']:.1f}%")
                            
                            if stats['avg_diff_ms'] <= 1.0:
                                quality = "✓ Excellent sync"
                            elif stats['avg_diff_ms'] <= 5.0:
                                quality = "✓ Good sync"
                            elif stats['avg_diff_ms'] <= 10.0:
                                quality = "⚠ Acceptable sync"
                            else:
                                quality = "❌ Poor sync"
                            print(f"  Quality: {quality}")
                            print()
                        
                        print(f"Total sync anomalies: {report['total_anomalies']}")
                    else:
                        print("No sync data available yet...")
                    
                    print("="*60)
                    last_report_time = current_time
                
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nUser interrupted (Ctrl+C).")
    except Exception as e:
        print(f"Error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nShutting down...")
        device_manager.cleanup()
        print("Cleanup complete.")

if __name__ == "__main__":
    main()