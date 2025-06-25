# Project Aria Development Environment

A containerized development environment for Meta's Project Aria smart glasses, enabling real-time stereo camera streaming, computer vision processing, and robotics integration.

## Quick Start

### Prerequisites
- Docker and Docker Compose installed
- Meta Project Aria smart glasses

### Setup and Development

1. **Run system diagnostics on HOST** (important first step):
   ```bash
   python doctor.py
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

- **Real-time Streaming**: UDP and ZeroMQ transport for stereo camera data
- **Computer Vision**: Camera calibration, undistortion, and image processing
- **ROS Integration**: ROS1 nodes for robotics applications
- **Containerized Development**: Reproducible environment with hardware access
- **Comprehensive Testing**: Built-in diagnostics and validation tools

## Core Components

- **Streaming System**: Multiple transport protocols (UDP, ZeroMQ)
- **Calibration Management**: Camera calibration and undistortion pipeline
- **Device Management**: Aria device connection and session handling
- **Analysis Tools**: Timestamp synchronization and performance monitoring

## Development Workflow

### Architecture Overview
- **Sender**: Runs inside Docker container (connects to Aria device)
- **Receiver**: Runs on HOST system (receives streamed data)

### Start Streaming (Inside Container)
```bash
# Enter container
docker exec -it aria-development bash

# Start UDP streaming
python udp_sender.py

# Start ZeroMQ streaming
python zmq_sender_usb.py
```

### Receive Data (On HOST)
```bash
# UDP receiver (on host)
python aria_workspace/udp_receiver.py

# ZeroMQ receiver (on host)
python aria_workspace/zmq_receiver.py
```

### ROS Integration (On HOST)
```bash
# Start ROS nodes (on host)
python aria_workspace/udp_receiver_node.py
python aria_workspace/zmq_receiver_node.py
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
   - **UDP Transport**: Basic streaming with JPEG compression
   - **ZeroMQ Transport**: High-performance IPC streaming
   - **ROS Integration**: ROS1 nodes for robotics applications

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