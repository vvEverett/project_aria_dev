# Dockerfile for Project Aria Development with ROS2 Core
# Base Image: ROS2 Humble Core instead of Ubuntu 22.04
FROM ros:humble-ros-core

# --- Environment Configuration ---
# Set non-interactive mode for package installations to prevent prompts.
ENV DEBIAN_FRONTEND=noninteractive
# Set a default timezone.
ENV TZ=Asia/Singapore

# --- System Dependencies Installation ---
# Update package lists and install necessary system libraries and tools.
# This includes Python, build tools, USB/ADB tools, critical GUI libraries, and ROS2 packages.
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Python essentials (keeping original structure)
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    # Build tools
    build-essential \
    cmake \
    pkg-config \
    # Additional ROS2 packages for communication
    ros-humble-rclpy \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-cv-bridge \
    # ROS2 build tools
    python3-colcon-common-extensions \
    python3-rosdep \
    # Hardware interaction tools
    usbutils \
    adb \
    libusb-1.0-0-dev \
    # Common utilities
    wget \
    curl \
    git \
    vim \
    nano \
    sudo \
    # CRITICAL: GUI libraries for OpenCV/Qt (cv2.imshow, etc.)
    libsm6 \
    libxext6 \
    libxrender1 \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglib2.0-0 \
    # Clean up apt cache to reduce image size
    && rm -rf /var/lib/apt/lists/*

# --- User Setup ---
# Create a non-root user 'developer' for better security and to avoid permission issues.
# Grant passwordless sudo privileges to this user and add to video group for camera access.
RUN useradd -m -s /bin/bash developer && \
    echo "developer ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    usermod -a -G video developer

# Switch to the new user for subsequent commands.
USER developer
WORKDIR /home/developer

# --- Python Virtual Environment Setup ---
# Create a virtual environment to isolate Python dependencies.
# Use --system-site-packages to access ROS2 Python packages.
RUN python3 -m venv /home/developer/venv --system-site-packages

# Add the virtual environment's bin directory to the PATH.
# This ensures 'pip' and 'python' commands use the venv's executables.
ENV PATH="/home/developer/venv/bin:$PATH"

# --- ROS2 Environment Setup ---
# Source ROS2 setup in bashrc for interactive sessions
RUN echo "source /opt/ros/humble/setup.bash" >> /home/developer/.bashrc

# Upgrade pip and install wheel for better package management.
# This is a separate layer for better caching.
RUN pip install --upgrade pip setuptools wheel --no-cache-dir

# Install the main Python packages: Project Aria SDK, ZMQ, OpenCV, and Numpy.
RUN pip install projectaria_client_sdk pyzmq opencv-python numpy --no-cache-dir

# Extract the Project Aria SDK sample files.
RUN python -m aria.extract_sdk_samples --output /home/developer

# Install the dependencies required by the SDK samples.
# Using a full path is more robust than 'cd ... && ...'.
RUN pip install -r /home/developer/projectaria_client_sdk_samples/requirements.txt --no-cache-dir

# --- ROS2 Workspace Setup ---
# Create a dedicated ROS2 workspace directory for Aria integration
RUN mkdir -p /home/developer/aria_ros_ws/src

# --- Workspace and Final Configuration ---
# Create a dedicated workspace directory (keeping original structure).
RUN mkdir -p /home/developer/aria_workspace

# Configure the shell to automatically activate the virtual environment for interactive sessions.
RUN echo "source /home/developer/venv/bin/activate" >> /home/developer/.bashrc

# Set the final working directory for the container.
WORKDIR /home/developer/aria_workspace

# Expose common ports used for Aria streaming and ROS2 communication.
EXPOSE 7000-8000 11311

# Default command - the environment is already configured in .bashrc
CMD ["/bin/bash"]