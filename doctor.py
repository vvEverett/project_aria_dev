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

import argparse
import hashlib
import logging
import os
import platform
import re
import stat
import subprocess
import sys

import tempfile
from dataclasses import dataclass
from typing import Callable, List, Optional

from termcolor import colored


def running_as_root() -> bool:
    return os.getuid() == 0


def run_command(cmd: List[str]) -> subprocess.CompletedProcess:
    logging.debug("Running: %s", " ".join(cmd))
    return subprocess.run(cmd, text=True, capture_output=True)


def failure(message: str):
    print(f"[ fail] {message}")
    exit(1)


def success(message: str):
    pill = colored("[ pass]", "white", "on_green", attrs=["bold"])
    message = colored(message, "green")
    print(pill, message)


def warning(message: str):
    pill = colored("[ warn]", "black", "on_yellow", attrs=["bold"])
    message = colored(message, "yellow")
    print(pill, message)


def action(message: str):
    print(f"    [+] {message}")


@dataclass
class Command:
    description: str
    cmd: List[str]


@dataclass
class File:
    description: str
    platforms: List[str]
    name: str
    path: str
    contents: str
    hash_suffix: bool = False
    permissions: Optional[int] = None
    owner: Optional[str] = None
    post_command: Optional[Command] = None

    def hash(self) -> str:
        return hashlib.sha1(self.contents.encode("utf8")).hexdigest()

    def file_path(self) -> str:
        if self.hash_suffix:
            name = f"{self.name}-{self.hash()}"
        else:
            name = self.name
        return os.path.join(self.path, name)

    def _read(self) -> str:
        with open(self.file_path(), "r") as f:
            return f.read()

    def _is_correct(self) -> bool:
        filepath = self.file_path()
        if not os.path.exists(filepath):
            return False
        # If we hash the file, no need to read contents
        if self.hash_suffix:
            return True
        if self.permissions is not None:
            st = os.stat(filepath)
            observed_permissions = stat.S_IMODE(st[stat.ST_MODE])
            if observed_permissions != self.permissions:
                logging.debug(
                    "Expected mode %s, got %s",
                    oct(self.permissions),
                    oct(observed_permissions),
                )
                return False
        try:
            d = self._read()
            if d != self.contents:
                logging.debug("File `%s` contents are outdated", filepath)
                return False
        except PermissionError:
            warning(f"Could not check {self.description} file, please re-run as sudo")
        return True

    def ensure(
        self,
    ):
        if platform.system() not in self.platforms:
            return

        filepath = self.file_path()

        def remediation():
            if not os.path.exists(self.path):
                failure(
                    f"Path `{self.path}` not found, please use a supported Linux distribution"
                )

            action(f"Writing {self.description} at {filepath}")

            if self.owner == "root":
                # Remove any files with outdated hashes
                if self.hash_suffix:
                    for file in os.listdir(self.path):
                        if os.path.isfile(
                            os.path.join(self.path, file)
                        ) and file.startswith(self.name + "-"):
                            action(f"Removing old {self.description} file {file}")
                            run_command(["sudo", "rm", os.path.join(self.path, file)])
                # Write file and copy to correct location
                with tempfile.NamedTemporaryFile("w") as f:
                    f.write(self.contents)
                    f.flush()
                    run_command(["sudo", "cp", f.name, filepath])
                    if self.permissions is not None:
                        run_command(
                            ["sudo", "chmod", oct(self.permissions)[2:], filepath]
                        )
            else:
                with open(filepath, "w") as f:
                    f.write(self.contents)
                if self.permissions is not None:
                    os.chmod(filepath, self.permissions)

            if self.post_command is not None:
                action(self.post_command.description)
                run_command(self.post_command.cmd)

        if not self._is_correct():
            remediate(
                f"{self.description} is not configured correctly.",
                "update the configuration",
                remediation,
            )

        success(f"{self.description} ok")


REQUIRED_CONFIGURATION_FILES = [
    File(
        description="Aria udev rules",
        platforms=["Linux"],
        name="52-aria.rules",
        path="/etc/udev/rules.d",
        owner="root",
        permissions=0o644,
        post_command=Command(
            description="Reloading udev rules",
            cmd=["sudo", "udevadm", "control", "—reload-rules"],
        ),
        contents="""
SUBSYSTEM=="usb", ATTRS{idVendor}=="2833", MODE="0666"
SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="2833" ATTRS{idProduct}=="0300", NAME="aria"
SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="2833" ATTRS{idProduct}=="9001", NAME="oatmeal"
""",
    ),
] + [
    File(
        description=f"{device.capitalize()} network manager connection",
        platforms=["Linux"],
        name=device,
        path="/etc/NetworkManager/system-connections",
        permissions=0o600,
        hash_suffix=True,
        owner="root",
        post_command=Command(
            description="Reloading network manager connections",
            cmd=["sudo", "nmcli", "connection", "reload"],
        ),
        contents=f"""
[connection]
id={device.capitalize()}
interface-name={device}
autoconnect=true
type=ethernet

[ipv4]
method=auto
never-default=true

[ipv6]
method=auto
never-default=true
""",
    )
    for device in ["aria", "oatmeal"]
]

IPTABLES_RULES = [
    "-A INPUT -p tcp -m tcp --dport 7400:7432 -j ACCEPT",
    "-A INPUT -p udp -m udp --dport 7400:7432 -j ACCEPT",
    "-A INPUT -p udp -m udp --dport 5354 -j ACCEPT",
]


def remediate(message: str, action: str, remediation: Callable):
    print(f"[error] {message}")
    input_message = f"        Would you like to {action}? ([y]/n): "
    ans = input(input_message)
    if ans == "" or ans == "y":
        try:
            remediation()
        except Exception as e:
            logging.debug("Exception: %s", e)
            failure(
                f"Failed to {action}. Please check https://facebookresearch.github.io/projectaria_tools/docs/ARK/sdk/sdk_troubleshooting"
            )

    else:
        exit(1)


def check_glib() -> bool:
    FAILED_TO_VALIDATE = "Failed validate glibc version, please check you are using a supported Linux distribution"

    if platform.system() != "Linux":
        return True

    output = run_command(["ldd", "--version", "ldd"])

    if output.returncode != 0:
        failure(FAILED_TO_VALIDATE)

    m = re.search(r"ldd .+ (\d+).(\d+)", output.stdout)

    if not m:
        failure(FAILED_TO_VALIDATE)

    ldd_major = int(m.group(1))
    ldd_minor = int(m.group(2))

    if ldd_major != 2 or ldd_minor < 34:
        failure(
            f"Your system glibc ({ldd_major}.{ldd_minor}) is too old, at least v2.34 is required. Please use a more recent Linux distribution"
        )

    success("glibc version ok")


def check_python_version():
    # Skipping the python version check when running on a known-good version
    if "ARIA_SKIP_PYTHON_VERSION_CHECK" in os.environ:
        return

    ver = sys.version_info
    if ver.major < 3 or (
        ver.major == 3 and (ver.minor < 8 or (ver.minor == 8 and ver.micro < 10))
    ):
        failure(
            f"Your version of Python ({ver.major}.{ver.minor}.{ver.micro}) is too old, at least 3.8.10 is required. Please update your python version."
        )
    success("Python version ok")


def check_macos_network():
    """
    On MacOS, check that the `Aria` network service is last in the service order.
    This ensures that Aria USB streaming does not prevent internet access
    via other services (e.g. Wi-Fi or ethernet)
    """

    if platform.system() != "Darwin":
        return

    response = run_command(["networksetup", "-listnetworkserviceorder"])

    if response.returncode != 0:
        failure("Failed to list network interfaces.")

    interfaces = [
        match.group(1)
        for match in re.finditer(r"^\(\d+\) (.+)$", response.stdout, flags=re.MULTILINE)
    ]

    ARIA_INTERFACES = ["Aria", "Oatmeal"]

    if not any(aria_interface in interfaces for aria_interface in ARIA_INTERFACES):
        warning(
            "No Aria network interface found, this will be created after you first start streaming. "
            "If you lose internet access when starting streaming over USB, please run `aria-doctor` "
            "again to restore connectivity."
        )
        return

    def remediation():
        # Move the Aria interface to lowest priority
        for aria_interface in ARIA_INTERFACES:
            if aria_interface in interfaces:
                interfaces.remove(aria_interface)
                interfaces.append(aria_interface)

        response = run_command(
            ["sudo", "networksetup", "-ordernetworkservices"] + interfaces
        )

        if response.returncode != 0:
            logging.debug(
                "networksetup failed: %i, %d", response.returncode, response.stdout
            )
            failure("Failed to set Aria network interface priority")

    # Check that there are no interfaces with lower priority than Aria/Oatmeal interfaces
    found_aria = False
    for interface in interfaces:
        if interface in ARIA_INTERFACES:
            found_aria = True
        elif found_aria:
            remediate(
                "Aria network interface priority is too high, this may interrupt other network connectivity while streaming",
                "lower the priority of the Aria network interface",
                remediation,
            )
            break

    success("Aria network interface priority ok")


def install_iptables_rules(rules):
    for rule in rules:
        run_command(["sudo", "iptables"] + rule.split(" "))


def check_iptables_rules():
    """
    On Linux, check that the iptables firewall rules are configured correctly.
    """

    if platform.system() != "Linux":
        return

    if not running_as_root():
        warning("Please run `sudo aria-doctor` to check iptables firewall rules.")
        return

    # Check that the iptables firewall rules are configured correctly
    response = run_command(["sudo", "iptables", "-S"])
    if response.returncode != 0:
        failure("Failed to list iptables rules.")
        return

    rules = response.stdout.split("\n")
    required_rules = [rule for rule in IPTABLES_RULES if rule not in rules]
    if required_rules:
        remediate(
            "iptables firewall rules are not configured correctly",
            "update the firewall rules",
            lambda: install_iptables_rules(required_rules),
        )

    success("iptable rules ok")


def main():
    parser = argparse.ArgumentParser(description="Project Aria Client SDK doctor.")
    parser.add_argument(
        "-v", "--verbose", help="increase output verbosity", action="store_true"
    )
    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)

    check_glib()
    check_python_version()
    check_macos_network()
    check_iptables_rules()

    for configuration_file in REQUIRED_CONFIGURATION_FILES:
        configuration_file.ensure()


if __name__ == "__main__":
    main()
