# Project Aria Development Environment

A containerized development environment for Meta's Project Aria smart glasses, enabling real-time stereo camera streaming, computer vision processing, and robotics integration.

## Quick Start

### Prerequisites
- Docker and Docker Compose installed
- Meta Project Aria smart glasses

### Setup and Development

1. **Run system diagnostics on HOST** (important first step):
   ```bash
   sudo python doctor.py
   ```
   This validates your system configuration and hardware connectivity.

2. **Build the development container**:
   ```bash
   docker compose build
   ```

3. **Start the development environment**:
   ```bash
   docker compose up
   ```

4. **Enter the development container**:
   ```bash
   docker exec -it aria-development bash
   ```
5. **Authorize Aria device** (inside container):
   ```bash
   python device_auth.py
   ```

6. **Verify Aria device connectivity** (inside container):
   ```bash
   python aria_usb_test.py
   ```

## Features

- **Computer Vision**: Camera calibration, undistortion, and image processing
- **ROS2 Integration**: Modern ROS2 nodes for robotics applications
- **Direct Display**: Native USB connection with OpenCV display
- **Containerized Development**: Reproducible environment with hardware access
- **Comprehensive Testing**: Built-in diagnostics and validation tools

## Core Components

- **ROS2 Integration**: Modern ROS2 publisher/subscriber nodes
- **Direct Display**: USB-based real-time camera display
- **Calibration Management**: Camera calibration and undistortion pipeline
- **Device Management**: Aria device connection and session handling
- **Analysis Tools**: Timestamp synchronization and performance monitoring

## Development Workflow

### Architecture Overview
- **Direct_Display**: Runs inside Docker container (connects to Aria device)
- **Sender(Publisher)**: Runs inside Docker container (connects to Aria device)
- **Receiver(Subscriber)**: Runs on HOST system or other docker container(receives streamed data)

### Start Streaming (Inside Container)

#### Current Methods
```bash
# Enter container
docker exec -it aria-development bash

# Direct display with real-time viewing
python aria_direct_display.py

# ROS2 publisher (with display)
python aria_ros2_publisher.py
```

#### Legacy Methods (Still Available)
```bash
# Legacy UDP streaming
python legacy/udp_sender.py

# Legacy ZeroMQ streaming
python legacy/zmq_sender_usb.py
```

### Receive Data (On HOST)

#### Current Methods
```bash
# ROS2 subscriber (all cameras in one window)
python aria_workspace/aria_ros2_subscriber.py
```

#### Legacy Methods (Still Available)
```bash
# Legacy UDP receiver
python aria_workspace/legacy/udp_receiver.py

# Legacy ZeroMQ receiver
python aria_workspace/legacy/zmq_receiver.py
```

### ROS Integration (On HOST)

#### ROS2 (Current)
```bash
# ROS2 subscriber node
python aria_workspace/aria_ros2_subscriber.py
```

#### ROS1 Legacy (Still Available)
```bash
# Legacy ROS1 nodes
python aria_workspace/legacy/udp_receiver_node.py
python aria_workspace/legacy/zmq_receiver_node.py
```

### Testing and Analysis (Inside Container)
```bash
# Test image undistortion
python aria_undistort_rgb_test.py

# Extract calibration data
python aria_calibration_extractor.py

# Analyze stream timing
python aria_stream_timestamp_sync_analyzer.py
```

## New Features (ROS2 & Direct Display)

### Direct Display Mode
- **aria_direct_display.py**: Direct USB connection with real-time OpenCV display
- **Features**: SLAM cameras (left/right) + RGB, undistortion toggle, frame saving
- **Controls**: 'q' quit, 's' save frames, 'u' toggle undistortion, ESC quit

### ROS2 Integration
- **aria_ros2_publisher.py**: Publishes all camera feeds to ROS2 topics
- **aria_ros2_subscriber.py**: Subscribes and displays all feeds in single window
- **Topics**: 
  - `/aria/slam_left/raw` and `/aria/slam_left/undistorted`
  - `/aria/slam_right/raw` and `/aria/slam_right/undistorted`  
  - `/aria/rgb/raw` and `/aria/rgb/undistorted`
- **Transport**: High-performance shared memory (SHM)

### Legacy Functions (Still Available)
Legacy streaming functions have been moved to `aria_workspace/legacy/` but remain fully functional:
- UDP streaming: `udp_sender.py`, `udp_receiver.py`, `udp_receiver_node.py`
- ZeroMQ streaming: `zmq_sender*.py`, `zmq_receiver*.py`, `zmq_receiver_node.py`
- Legacy utilities: `aria_utils.py`, `common.py`

## Configuration

- **Default Aria IP**: 192.168.1.25
- **Streaming Ports**: 7000-8000
- **Calibration File**: `device_calibration.json`
- **Working Directory**: `/home/developer/aria_workspace`

## Troubleshooting

1. **Device not found**: Run `python doctor.py` on host to check USB connectivity
2. **Permission issues**: Ensure container has privileged access
3. **Network problems**: Verify Aria device IP and network configuration
4. **GUI issues**: Check X11 forwarding setup

## Architecture

### Core Components

1. **AriaDeviceManager** (`aria_workspace/aria_utils.py`):
   - Device connection and streaming configuration
   - Session management and cleanup
   - Calibration data loading and saving

2. **CalibrationManager** (`aria_workspace/aria_utils.py`):
   - Camera calibration and undistortion
   - Multi-camera support (SLAM left/right, RGB)
   - Configurable output parameters

3. **Streaming System**:
   - **Direct Display**: Real-time USB connection with OpenCV display
   - **ROS2 Integration**: Modern ROS2 publisher/subscriber with SHM transport
   - **Legacy Support**: UDP/ZeroMQ streaming and ROS1 nodes (in legacy/ folder)

### Data Flow Architecture

```
Aria Device → Device Manager → Streaming Observer → Transport Layer → Receiver/Node
                ↓
            Calibration Manager → Image Processing → Output
```

### Key Classes and Patterns

- **StreamingObserver**: Base class for all streaming implementations
- **CalibrationManager**: Handles camera calibration and undistortion
- **AriaDeviceManager**: Manages device lifecycle and configuration
- **Transport Abstraction**: UDP vs ZeroMQ implementations share common interfaces

### Development Patterns

- **Error Handling**: All modules implement comprehensive error handling with graceful fallbacks and detailed logging
- **Resource Management**: Use context managers and proper cleanup in finally blocks. The `AriaDeviceManager` handles session cleanup automatically
- **Threading and Concurrency**: Streaming observers run in separate threads. Use proper synchronization when accessing shared resources

## License

Licensed under the Apache License, Version 2.0. See individual source files for copyright notices.