# Dockerfile for Project Aria Development
# Base Image: Ubuntu 22.04 LTS
FROM ubuntu:22.04

# --- Environment Configuration ---
# Set non-interactive mode for package installations to prevent prompts.
ENV DEBIAN_FRONTEND=noninteractive
# Set a default timezone.
ENV TZ=Asia/Singapore

# --- System Dependencies Installation ---
# Update package lists and install necessary system libraries and tools.
# This includes Python, build tools, USB/ADB tools, and critical GUI libraries.
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Python essentials
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    # Build tools
    build-essential \
    cmake \
    pkg-config \
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
# Grant passwordless sudo privileges to this user.
RUN useradd -m -s /bin/bash developer && \
    echo "developer ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to the new user for subsequent commands.
USER developer
WORKDIR /home/developer

# --- Python Virtual Environment Setup ---
# Create a virtual environment to isolate Python dependencies.
RUN python3 -m venv /home/developer/venv

# Add the virtual environment's bin directory to the PATH.
# This ensures 'pip' and 'python' commands use the venv's executables.
ENV PATH="/home/developer/venv/bin:$PATH"

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

# --- Workspace and Final Configuration ---
# Create a dedicated workspace directory.
RUN mkdir -p /home/developer/aria_workspace

# Configure the shell to automatically activate the virtual environment for interactive sessions.
RUN echo "source /home/developer/venv/bin/activate" >> /home/developer/.bashrc

# Set the final working directory for the container.
WORKDIR /home/developer/aria_workspace

# Expose common ports used for Aria streaming (for documentation and '-P' flag).
EXPOSE 7000-8000

# The default command when starting the container is to open a bash shell.
CMD ["/bin/bash"]

