from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from lerobot.robots.config import RobotConfig

@RobotConfig.register_subclass("lerobot_xarm")
@dataclass
class XarmConfig(RobotConfig):
    ip: str = "192.168.1.184"
    save_effort: bool = False
    limit_joints: bool = False
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    device_class: str = 'lerobot_xarm.xarm.Xarm'
