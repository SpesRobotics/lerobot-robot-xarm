from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from lerobot.robots.config import RobotConfig

@RobotConfig.register_subclass("lerobot_robot_xarm")
@dataclass
class XarmConfig(RobotConfig):
    ip: str = "192.168.1.184"
    use_effort: bool = False
    use_velocity: bool = True
    use_acceleration: bool = True
    home_translation: list[float] = field(default_factory=lambda: [0.2, 0.0, 0.05])
    home_orientation_euler: list[float] = field(default_factory=lambda: [3.14, 0.0, 0.0])
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
