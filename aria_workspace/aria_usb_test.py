#!/usr/bin/env python3
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
import aria.sdk as aria
import cv2
import numpy as np

def main():
    print("=== Aria Glass Minimal USB Connection ===")
    
    # Set SDK's log level
    aria.set_log_level(aria.Level.Info)
    
    try:
        # 1. Create and connect
        print("\n1. Connecting to Aria device via USB...")
        device_client = aria.DeviceClient()
        device = device_client.connect()
        print("   Connection successful!")
        
        # 2. Print device information
        print("\n2. Device information:")
        info = device.info
        status = device.status
        print(f"   Model: {info.model}")
        print(f"   Serial: {info.serial}")
        print(f"   Battery: {status.battery_level}%")
        print(f"   Mode: {status.device_mode}")
        
        # 3. Get streaming and recording managers
        streaming_manager = device.streaming_manager
        recording_manager = device.recording_manager
        
        # Check current states
        streaming_state = streaming_manager.streaming_state
        recording_state = recording_manager.recording_state
        print(f"\n3. Current states:")
        print(f"   Streaming state: {streaming_state}")
        print(f"   Recording state: {recording_state}")
        
        # 4. Try to stop streaming and recording if needed
        print("\n4. Attempting to stop any active sessions...")
        try:
            streaming_manager.stop_streaming()
            print("   Streaming stopped (or was already stopped)")
        except Exception as e:
            print(f"   Note: {e}")
            
        try:
            recording_manager.stop_recording()
            print("   Recording stopped (or was already stopped)")
        except Exception as e:
            print(f"   Note: {e}")
        
        # Give device time to process
        print("   Waiting for device to process...")
        time.sleep(3)
        
        # 5. Configure streaming for USB
        print("\n5. Setting up USB streaming...")
        streaming_config = aria.StreamingConfig()
        streaming_config.profile_name = "profile18"
        streaming_config.streaming_interface = aria.StreamingInterface.Usb
        streaming_config.security_options.use_ephemeral_certs = True
        streaming_manager.streaming_config = streaming_config
        
        # 6. Start streaming
        print("\n6. Starting streaming...")
        try:
            streaming_manager.start_streaming()
            print("   Streaming requested successfully")
            
            # Check the state after starting
            time.sleep(2)
            state = streaming_manager.streaming_state
            print(f"   Current streaming state: {state}")
        except Exception as e:
            print(f"   Error starting streaming: {e}")
            print("   Trying a manual approach to resolve active sessions...")
            
            # Try an alternative approach if the first fails
            print("\n   Alternative approach: Please press the capture button on your")
            print("   Aria device to manually stop any active sessions.")
            print("   Waiting 10 seconds for you to press the button...")
            for i in range(10, 0, -1):
                print(f"   {i} seconds remaining...", end="\r", flush=True)
                time.sleep(1)
            print("\n   Trying to start streaming again...")
            
            try:
                streaming_manager.start_streaming()
                print("   Streaming started successfully!")
            except Exception as e:
                print(f"   Still encountering error: {e}")
                print("   Unable to start streaming. Please restart your device and try again.")
                device_client.disconnect(device)
                return
        
        # 7. Set up a basic streaming client
        print("\n7. Setting up streaming client...")
        streaming_client = streaming_manager.streaming_client
        
        class SimpleObserver:
            def __init__(self):
                self.rgb_image = None
            
            def on_image_received(self, image, record):
                if record.camera_id == aria.CameraId.Rgb:
                    self.rgb_image = image
        
        observer = SimpleObserver()
        
        # Configure what data to receive
        config = streaming_client.subscription_config
        config.subscriber_data_type = aria.StreamingDataType.Rgb
        streaming_client.subscription_config = config
        
        # Start receiving data
        streaming_client.set_streaming_client_observer(observer)
        streaming_client.subscribe()
        
        # 8. Display video feed
        print("\n8. Attempting to display video feed. Press 'q' or ESC to exit.")
        print("   (If no video appears, streaming may not be working correctly)")
        window_name = "Aria USB Camera"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 800, 800)
        
        # Main display loop
        start_time = time.time()
        duration = 60  # Run for 60 seconds max
        
        while time.time() - start_time < duration:
            # Check for exit key
            key = cv2.waitKey(1)
            if key == 27 or key == ord('q'):  # ESC or 'q'
                break
                
            # Display the image if available
            if observer.rgb_image is not None:
                # Rotate and convert color format
                rgb_image = np.rot90(observer.rgb_image, -1)
                rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
                cv2.imshow(window_name, rgb_image)
        
        # 9. Clean up
        print("\n9. Cleaning up...")
        cv2.destroyAllWindows()
        streaming_client.unsubscribe()
        
        try:
            streaming_manager.stop_streaming()
            print("   Streaming stopped")
        except Exception as e:
            print(f"   Note when stopping streaming: {e}")
            
        device_client.disconnect(device)
        print("Session ended.")
        
    except Exception as e:
        print(f"\nError: {e}")
        print("Disconnecting...")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()