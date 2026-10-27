#!/usr/bin/env python3
"""Installation/ABI inventory only: no ROS node, simulation step, or robot driver."""
import argparse
import json
import os
import platform
import sys
from importlib.metadata import version
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


def check():
    from ament_index_python.packages import get_package_prefix, get_package_share_directory

    packages = (
        "controller_manager", "hardware_interface", "joint_trajectory_controller",
        "joint_state_broadcaster", "position_controllers", "control_toolbox",
        "control_msgs", "mujoco_ros2_control", "mujoco_ros2_control_demos",
        "mujoco_vendor", "mujoco_ros2_control_msgs", "mujoco_ros2_control_plugins",
    )
    versions = {}
    for name in packages:
        manifest = Path(get_package_share_directory(name)) / "package.xml"
        versions[name] = ET.parse(manifest).findtext("version")
    for name, expected in {
        "mujoco_ros2_control": "0.1.2",
        "mujoco_ros2_control_demos": "0.1.2",
        "mujoco_vendor": "0.1.0",
        "mujoco_ros2_control_msgs": "0.1.2",
        "mujoco_ros2_control_plugins": "0.1.2",
    }.items():
        if versions[name] != expected:
            raise RuntimeError(f"{name}: expected {expected}, got {versions[name]}")

    vendor_prefix = Path(get_package_prefix("mujoco_vendor"))
    library = vendor_prefix / "opt/mujoco_vendor/lib/libmujoco.so"
    # Probe the vendor in a separate process, avoiding resolution against a pip library.
    probe = "import ctypes,sys; m=ctypes.CDLL(sys.argv[1]); m.mj_versionString.restype=ctypes.c_char_p; print(m.mj_versionString().decode())"
    engine_version = subprocess.check_output(
        ["/usr/bin/python3", "-c", probe, str(library)], text=True).strip()
    if engine_version != "3.12.0":
        raise RuntimeError(f"Expected vendor MuJoCo 3.12.0, got {engine_version}")

    bridge = Path(get_package_prefix("mujoco_ros2_control"))
    executable = bridge / "lib/mujoco_ros2_control/ros2_control_node"
    if not os.access(executable, os.X_OK):
        raise RuntimeError(f"Missing executable: {executable}")
    linked = subprocess.run(["ldd", str(executable)], text=True, capture_output=True, check=True)
    if "not found" in linked.stdout:
        raise RuntimeError(linked.stdout)
    demo = Path(get_package_share_directory("mujoco_ros2_control_demos"))
    launch_paths = [demo / "launch" / name for name in
                    ("01_basic_robot.launch.py", "03_pid_control.launch.py")]
    for path in launch_paths:
        if not path.is_file():
            raise RuntimeError(f"Missing official demo: {path}")
    if sys.version_info[:2] != (3, 12):
        raise RuntimeError(f"Expected Python 3.12, got {sys.version}")
    if platform.freedesktop_os_release()["VERSION_ID"] != "24.04":
        raise RuntimeError("Expected Ubuntu 24.04")
    if os.environ.get("ROS_DISTRO") != "jazzy":
        raise RuntimeError("Expected ROS 2 Jazzy")
    # Importing ROS messages checks Python bindings; no rclpy.init()/node creation.
    import rclpy
    from control_msgs.action import FollowJointTrajectory
    from sensor_msgs.msg import JointState
    import mujoco
    import numpy as np
    import torch
    import scipy
    import matplotlib
    import yaml
    expected_python = {"mujoco": "3.12.0", "numpy": "1.26.4", "scipy": "1.14.1",
                       "matplotlib": "3.9.4", "PyYAML": "6.0.2", "torch": "2.9.1+cpu"}
    for name, expected in expected_python.items():
        if version(name) != expected:
            raise RuntimeError(f"{name}: expected {expected}, got {version(name)}")
    if mujoco.mj_versionString() != engine_version:
        raise RuntimeError("Python and ROS vendor MuJoCo versions differ")
    if torch.version.cuda is not None:
        raise RuntimeError("Expected CPU PyTorch")
    x = torch.tensor([1.0, 2.0], dtype=torch.float64, requires_grad=True)
    x.square().sum().backward()
    np.testing.assert_allclose(x.grad.numpy(), [2., 4.], rtol=0, atol=0)
    return {
        "python_package_versions": expected_python,
        "python_mujoco_library_version": mujoco.mj_versionString(),
        "pytorch_cpu_autograd_checked": True,
        "scope": "installation_only", "ros_distro": os.environ.get("ROS_DISTRO"),
        "package_versions": versions, "mujoco_version": engine_version,
        "mujoco_library": str(library.resolve()), "bridge_executable": str(executable),
        "link_dependencies": linked.stdout.splitlines(),
        "official_demo_launches": [str(p) for p in launch_paths],
        "physics_steps": 0, "ros_nodes_started": 0,
        "controller_activation_verified": False,
        "ur5_tracking_verified": False,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = check()
    text = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text)
    print(text, end="")
