# Project Aria Dockerized Development Environment

## Overview
This project provides a pre-configured Docker-based development environment for working with the Project Aria SDK. It leverages VS Code Dev Containers for seamless integration, offering a consistent and isolated environment with all necessary dependencies and tools for Aria development.

## Features
- ✅ Ubuntu 22.04 environment (compatible with Project Aria SDK)
- ✅ Python virtual environment pre-configured
- ✅ Full VS Code integration with debugging and auto-completion
- ✅ USB device access support for Aria glasses
- ✅ Network streaming support
- ✅ Includes Project Aria SDK tools and sample access

## Prerequisites
Before you begin, ensure you have the following installed:
- Docker Desktop (or Docker Engine + Docker Compose)
- Visual Studio Code
- VS Code "Dev Containers" extension (ID: `ms-vscode-remote.remote-containers`)

## Getting Started

1.  **Clone this Repository:**
    If you haven't already, clone this repository to your local machine:
    ```bash
    # Replace <repository-url> with the actual URL if applicable
    git clone <repository-url> 
    cd project_aria_dev
    ```

2.  **Launch the Development Environment:**
    You have two main options:

    *   **Option A: Using the `start_dev.sh` script (Recommended for initial setup & USB passthrough)**
        This script helps configure necessary settings, especially for USB device access.
        From the `project_aria_dev` directory on your host machine:
        ```bash
        ./scripts/start_dev.sh
        ```
        After the script completes, open the `project_aria_dev` folder in VS Code. VS Code should detect the `.devcontainer` configuration and prompt you to "Reopen in Container". Click this button.

    *   **Option B: Directly with VS Code**
        Open the `project_aria_dev` folder in VS Code.
        VS Code will detect the `.devcontainer/devcontainer.json` file. Click the "Reopen in Container" button in the notification popup, or open the Command Palette (Ctrl+Shift+P or Cmd+Shift+P) and run the "Dev Containers: Reopen in Container" command.

3.  **Wait for Container Build:**
    The first time you launch the environment, Docker will build the container image. This may take several minutes. Subsequent launches will be significantly faster as Docker caches the layers.

4.  **Verify Setup:**
    Once VS Code has connected to the container, open a new terminal within VS Code (Terminal > New Terminal). You can verify the environment:
    *   Check Aria SDK tools:
        ```bash
        aria-doctor
        ```
    *   Test a simple connection script (if `test_connection.py` exists and is relevant):
        ```bash
        python aria_workspace/test_connection.py
        ```

## Project Structure
```
project_aria_dev/
├── .devcontainer/          # VS Code Dev Container Configuration (devcontainer.json, Dockerfile)
├── aria_workspace/         # Your primary code workspace (mounted into the container)
├── scripts/               # Helper scripts (e.g., start_dev.sh, dev_setup.sh)
├── docker-compose.yml     # Docker Compose configuration for services
├── Dockerfile             # Base Dockerfile if not fully defined in .devcontainer
└── README.md              # This documentation file
```

## Working within the Environment

### Your Workspace (`aria_workspace/`)
-   The `aria_workspace/` directory is your main area for development. It's mounted directly from your host machine into the container.
-   Place your custom Project Aria applications, scripts, and data here.
-   Changes made in this directory within the container will be reflected on your host machine, and vice-versa.

### Connecting to Aria Glasses

1.  **USB Connection:**
    -   Connect your Aria device to your computer via USB.
    -   If you used `./scripts/start_dev.sh`, it should assist with USB device passthrough to the Docker container.
    -   Inside the container's terminal, you can check for Android Debug Bridge (ADB) connected devices:
        ```bash
        adb devices
        ```
        Your Aria device should be listed.

2.  **Wi-Fi Connection:**
    -   Ensure your Aria device and your computer (running Docker) are on the same Wi-Fi network.
    -   Follow the Project Aria SDK documentation for specific instructions on setting up and using network streaming.

### Running Project Aria SDK Samples
The Project Aria SDK and its samples are typically pre-installed or made available within the container. A common location for samples is `~/projectaria_client_sdk_samples`.
```bash
# Navigate to the SDK samples directory (path may vary)
cd ~/projectaria_client_sdk_samples

# Example: Run a device connection test sample
python device_connect.py

# List other available samples
ls -la
```
Refer to the official Project Aria SDK documentation for details on specific samples.

### Useful Commands & Scripts

-   **Inside the Container:**
    -   `aria-doctor`: Diagnoses the Project Aria SDK setup and device connectivity.
    -   `adb devices`: Lists devices connected via ADB (primarily for USB connections).
-   **On the Host Machine (in `project_aria_dev` directory):**
    -   `./scripts/start_dev.sh`: Starts the Docker Compose services, often configured for privileged mode to enable USB device access.
    -   `./scripts/dev_setup.sh`: (Purpose may vary) Typically used for initial one-time setup tasks for the development environment. Consult the script's content for its specific actions.

## Troubleshooting

1.  **USB Device Not Accessible in Container:**
    -   Ensure Docker is running with permissions to access USB devices. Using `start_dev.sh` often handles this by running the container in privileged mode.
    -   Verify udev rules on Linux hosts if issues persist.
    -   On Windows/macOS, ensure Docker Desktop has access to USB resources.

2.  **Network Streaming Issues:**
    -   Check your firewall settings on both the host machine and within your network. Ensure the necessary ports (e.g., 7000-8000, or as specified by Aria SDK) are open for TCP/UDP traffic.
    -   Confirm the Aria device and host computer are on the same subnet.

3.  **Permission Denied in Container:**
    -   The container typically runs as a non-root user (e.g., `vscode` or `developer`). Ensure files in mounted volumes have appropriate permissions.
    -   If you need to install software or perform privileged operations, you might need to use `sudo` within the container if configured, or adjust the Dockerfile.

## Customizing the Environment
You can customize the development environment by modifying the following files:
-   **`.devcontainer/devcontainer.json`**: Configure VS Code settings specific to the container, install VS Code extensions, define post-create commands, etc.
-   **`.devcontainer/Dockerfile` (or the main `Dockerfile`)**: Add or remove system packages, change base image, set environment variables, etc.
-   **`docker-compose.yml`**: Modify service definitions, port mappings, volume mounts, or add other services.

After making changes to these configuration files, you'll need to rebuild the container. In VS Code, open the Command Palette and run "Dev Containers: Rebuild Container".

## Support
For issues related to Project Aria SDK or hardware, please refer to:
-   [Project Aria Official Documentation](https://facebookresearch.github.io/projectaria_tools/)
-   [Project Aria SDK Troubleshooting Guide](https://facebookresearch.github.io/projectaria_tools/docs/ARK/sdk/sdk_troubleshooting)

For issues specific to this Dockerized environment setup, please check the repository's issue tracker or contact the maintainers.
