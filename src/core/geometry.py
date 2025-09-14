import math
from dataclasses import dataclass
from typing import Tuple

@dataclass
class Pose2D:
    x: float
    y: float
    theta: float

def quaternion_to_euler(x: float, y: float, z: float, w: float) -> Tuple[float,float,float]:
    sinr_cosp = 2*(w*x + y*z); cosr_cosp = 1 - 2*(x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2*(w*y - z*x)
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2*(w*z + x*y); cosy_cosp = 1 - 2*(y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float,float,float,float]:
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return x, y, z, w
