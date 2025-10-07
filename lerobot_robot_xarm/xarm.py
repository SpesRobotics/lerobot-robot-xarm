import logging
import time
import os
import copy
from typing import Any

import numpy as np
import transforms3d as t3d
from xarm.wrapper import XArmAPI
from teleop.utils.jacobi_robot import JacobiRobot

from lerobot.cameras import make_cameras_from_configs
from lerobot.utils.errors import DeviceNotConnectedError, DeviceAlreadyConnectedError
from lerobot.robots.robot import Robot

from .config_xarm import XarmConfig


logger = logging.getLogger(__name__)


class Lite6Gripper:
    def __init__(self, arm: XArmAPI):
        self._arm = arm
        self._prev_gripper_state = None
        self._gripper_state = 0.0
        self._gripper_open_time = 0.0
        self._gripper_stopped = False

    def open(self):
        self._arm.open_lite6_gripper()

    def close(self):
        self._arm.close_lite6_gripper()

    def stop(self):
        self._arm.stop_lite6_gripper()

    def set_gripper_state(self, gripper_state: float) -> None:
        self._gripper_state = gripper_state

        if self._gripper_state is not None:
            if (
                self._prev_gripper_state is None
                or self._prev_gripper_state != self._gripper_state
            ):
                if self._gripper_state < 1.0:
                    self._gripper_stopped = False
                    self.close()
                else:
                    self._gripper_open_time = time.time()
                    self._gripper_stopped = False
                    self.open()

            if (
                not self._gripper_stopped
                and self._gripper_state >= 1.0
                and time.time() - self._gripper_open_time > 1.0
            ):
                # If gripper was closed and now is open, stop the gripper
                self._gripper_stopped = True
                self.stop()

        self._prev_gripper_state = self._gripper_state

    def get_gripper_state(self) -> float:
        return self._gripper_state

    def reset_gripper(self) -> None:
        self._prev_gripper_state = None
        self._gripper_state = 0.0
        self._gripper_open_time = 0.0
        self._gripper_stopped = False


class Xarm(Robot):
    config_class = XarmConfig
    name = "xarm"

    def __init__(self, config: XarmConfig):
        super().__init__(config)
        self.cameras = make_cameras_from_configs(config.cameras)

        this_dir = os.path.dirname(os.path.abspath(__file__))

        self.config = config
        self._is_connected = False
        self._arm = None
        self._gripper = None
        self._jacobi = JacobiRobot(
            os.path.join(this_dir, "lite6.urdf"), ee_link="link_tool"
        )
        self._initial_pose = None
        self._prev_observation = None

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        # Connect to the robot and set it to a servo mode
        self._arm = XArmAPI("192.168.1.184", is_radian=True)
        self._arm.connect()
        self._arm.motion_enable(enable=True)
        self._arm.set_mode(1)  # Position mode
        self._arm.set_state(state=0)  # Sport state

        # Initialize gripper
        self._gripper = Lite6Gripper(self._arm)

        # set joint positions to jacobi, read from arm
        code, joint_positions = self._arm.get_servo_angle()
        if code != 0:
            raise DeviceNotConnectedError(f"Failed to get joint angles from {self}")
        for i in range(1, 7):
            joint_name = f"joint{i}"
            self._jacobi.set_joint_position(joint_name, joint_positions[i - 1])

        for cam in self.cameras.values():
            cam.connect()

        self.is_connected = True
        self.configure()
        logger.info(f"{self} connected.")

    @property
    def _motors_ft(self) -> dict[str, type]:
        motors = {f"joint{i}.pos": float for i in range(1, 7)}
        motors["gripper.pos"] = float
        return motors

    @property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        action = copy.deepcopy(action)

        if "delta_x" in action and "delta_y" in action and "delta_z" in action:
            pose = self._jacobi.get_ee_pose()
            delta_pose = np.eye(4)
            delta_pose[:3, 3] = [
                action["delta_x"],
                action["delta_y"],
                action["delta_z"],
            ]

            delta_rotation = None
            if (
                "delta_roll" in action
                and "delta_pitch" in action
                and "delta_yaw" in action
            ):
                roll = action["delta_roll"]
                pitch = action["delta_pitch"]
                yaw = action["delta_yaw"]
                delta_rotation = t3d.euler.euler2mat(roll, pitch, yaw)
                delta_pose[:3, :3] = delta_rotation

            action["pose"] = np.eye(4)
            if delta_rotation is not None:
                action["pose"][:3, :3] = delta_rotation @ pose[:3, :3]
            else:
                action["pose"][:3, :3] = pose[:3, :3]
            action["pose"][:3, 3] = pose[:3, 3] + delta_pose[:3, 3]

        if "pose_from_initial" in action:
            delta_pose = action["pose_from_initial"]
            action["pose"] = np.eye(4)
            action["pose"][:3, :3] = delta_pose[:3, :3] @ self._initial_pose[:3, :3]
            action["pose"][:3, 3] = self._initial_pose[:3, 3] + delta_pose[:3, 3]

        if "home" in action:
            home_pose = t3d.affines.compose(
                self.config.home_translation,
                t3d.euler.euler2mat(*self.config.home_orientation_euler),
                [1.0, 1.0, 1.0],
            )
            action["pose"] = home_pose
            self._initial_pose = self._jacobi.get_ee_pose()

        if "pose" in action:
            pose = action["pose"]
            self._jacobi.servo_to_pose(pose)

            # Get joint positions from Jacobi
            for i in range(1, 7):
                joint_pos = self._jacobi.get_joint_position(f"joint{i}")
                action[f"joint{i}.pos"] = joint_pos

        # Execute joint positions
        if "joint1.pos" in action:
            joint_positions = []
            for i in range(1, 7):
                joint_pos = action[f"joint{i}.pos"]
                joint_positions.append(joint_pos)

            self._arm.set_servo_angle_j(joint_positions)
        else:
            for i in range(1, 7):
                action[f"joint{i}.pos"] = self._jacobi.get_joint_position(f"joint{i}")

        # Send gripper command
        if "gripper.pos" in action or "gripper" in action:
            gripper_pos = action.get("gripper.pos", action.get("gripper", 0.0))
            self._gripper.set_gripper_state(gripper_pos)
            action["gripper.pos"] = self._gripper.get_gripper_state()

        return action

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Read arm position
        start = time.perf_counter()

        # Read joint positions from xarm
        _, (joint_angles, _, joint_efforts) = self._arm.get_joint_states()

        obs_dict = {}
        for i, angle in enumerate(joint_angles[:6]):  # First 6 angles are joints
            obs_dict[f"joint{i+1}.pos"] = angle
            obs_dict[f"joint{i+1}.effort"] = joint_efforts[i]
        obs_dict["gripper.pos"] = self._gripper.get_gripper_state()

        # calculate joint velocities
        for i in range(1, 7):
            joint_name = f"joint{i}"
            if self._prev_observation is not None:
                prev_angle = self._prev_observation[f"{joint_name}.pos"]
                curr_angle = obs_dict[f"{joint_name}.pos"]
                obs_dict[f"{joint_name}.vel"] = (
                    curr_angle - prev_angle
                ) * 60.0  # Assuming 60Hz control frequency
            else:
                obs_dict[f"{joint_name}.vel"] = 0.0

        # calculate joint accelerations
        for i in range(1, 7):
            joint_name = f"joint{i}"
            if self._prev_observation is not None:
                prev_vel = self._prev_observation[f"{joint_name}.vel"]
                obs_dict[f"{joint_name}.acc"] = obs_dict[f"{joint_name}.vel"] - prev_vel
            else:
                obs_dict[f"{joint_name}.acc"] = 0.0

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        self._prev_observation = obs_dict
        return obs_dict

    def reset(self):
        if self._gripper:
            self._gripper.reset_gripper()

    def disconnect(self) -> None:
        if not self.is_connected:
            return

        if self._gripper is not None:
            self._gripper.stop()
            self._gripper = None

        if self._arm is not None:
            self._arm.disconnect()
            self._arm = None

        for cam in self.cameras.values():
            cam.disconnect()

        self.is_connected = False
        logger.info(f"{self} disconnected.")

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self._initial_pose = self._jacobi.get_ee_pose()
        xyz = self._initial_pose[:3, 3]
        rpy = t3d.euler.mat2euler(self._initial_pose[:3, :3])
        print(f"Initial pose: xyz={xyz}, rpy={rpy}")

    def is_calibrated(self) -> bool:
        return self.is_connected

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @is_connected.setter
    def is_connected(self, value: bool) -> None:
        self._is_connected = value

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3)
            for cam in self.cameras
        }

    @property
    def observation_features(self) -> dict[str, Any]:
        features = {**self._motors_ft, **self._cameras_ft}
        if self.config.use_effort:
            for i in range(1, 7):
                features[f"joint{i}.effort"] = float
        if self.config.use_velocity:
            for i in range(1, 7):
                features[f"joint{i}.vel"] = float
        if self.config.use_acceleration:
            for i in range(1, 7):
                features[f"joint{i}.acc"] = float
        return features

    @property
    def cameras(self):
        return self._cameras

    @cameras.setter
    def cameras(self, value):
        self._cameras = value

    @property
    def config(self):
        return self._config

    @config.setter
    def config(self, value):
        self._config = value
