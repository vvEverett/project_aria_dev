# Dockerfile for Project Aria Development
FROM ubuntu:22.04

# Set non-interactive mode
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Singapore

# Install basic dependencies and OpenGL libraries
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    usbutils \
    adb \
    wget \
    curl \
    git \
    vim \
    nano \
    sudo \
    build-essential \
    cmake \
    pkg-config \
    libusb-1.0-0-dev \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Create development user (avoid permission issues)
RUN useradd -m -s /bin/bash developer && \
    echo "developer ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to development user
USER developer
WORKDIR /home/developer

# Create virtual environment
RUN python3 -m venv ~/venv

# Activate virtual environment and install Project Aria SDK + ZMQ
RUN /bin/bash -c "source ~/venv/bin/activate && \
    pip install --upgrade pip && \
    pip install projectaria_client_sdk --no-cache-dir && \
    pip install pyzmq opencv-python numpy --no-cache-dir && \
    python -m aria.extract_sdk_samples --output ~ && \
    cd ~/projectaria_client_sdk_samples && \
    pip install -r requirements.txt"

# Create workspace directory
RUN mkdir -p /home/developer/aria_workspace

# Set environment variables
ENV PATH="/home/developer/venv/bin:$PATH"

# Activate virtual environment by default
RUN echo "source ~/venv/bin/activate" >> ~/.bashrc

WORKDIR /home/developer/aria_workspace

# Expose common ports (for Aria streaming only, ZMQ uses IPC)
EXPOSE 7000-8000

CMD ["/bin/bash"]