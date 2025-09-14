# epuck_session.py
import math
import time
import re
from dataclasses import dataclass
from typing import Optional, Union, Tuple, List
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from src.config import EPUCK_PATH_HINTS

def quaternion_to_euler(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """Retorna (roll, pitch, yaw) em rad."""

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Recebe (roll, pitch, yaw) em rad e retorna (x,y,z,w)."""
    cy = math.cos(yaw * 0.5);  sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5);  sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)

@dataclass
class Pose2D:
    x: float
    y: float
    theta: float

class CoppeliaSimSession:
    def __init__(self, host: str = "localhost", port: int = 23000, auto_connect: bool = True):
        self.host = host
        self.port = port
        self.client: Optional[RemoteAPIClient] = None
        self.simulation = None
        if auto_connect:
            self.connect()

    def __enter__(self) -> "CoppeliaSimSession":
        if self.client is None:
            self.connect()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def connect(self):
        self.client = RemoteAPIClient(host=self.host, port=self.port)
        self.simulation = self.client.getObject("sim")
        # stepping explícito (se suportado)
        try:
            self.simulation.setStepping(True)
        except Exception:
            pass

    def close(self):
        self.simulation = None
        self.client = None

    def start(self):
        self._ensure_ready()
        self.simulation.startSimulation()

    def stop(self):
        self._ensure_ready()
        self.simulation.stopSimulation()

    def step(self):
        self._ensure_ready()
        self.simulation.step()

    def is_running(self) -> bool:
        self._ensure_ready()
        return self.simulation.getSimulationState() != self.simulation.simulation_stopped

    def wait_until_running(self, timeout_seconds: float = 5.0):
        self._ensure_ready()
        t0 = time.time()
        while not self.is_running():
            if time.time() - t0 > timeout_seconds:
                raise TimeoutError("[ERRO] A simulação não iniciou dentro do tempo limite.")
            time.sleep(0.05)

    def get_simulation_time(self) -> float:
        self._ensure_ready()
        return self.simulation.getSimulationTime()

    def get_object(self, path: str) -> int:
        self._ensure_ready()
        try:
            return self.simulation.getObject(path)
        except Exception as e:
            raise RuntimeError(f"[ERRO] Não foi possível obter objeto '{path}': {e}")

    # ---- helpers de constantes com fallback numérico ----
    def _const(self, name: str, default: int) -> int:
        return getattr(self.simulation, name, default)

    def _handle_world(self) -> int:
        return self._const("handle_world", -1)

    def _object_joint_type(self) -> int:
        return self._const("object_joint_type", 2)

    def _joint_revolute_subtype(self) -> int:
        return self._const("joint_revolute_subtype", 10)

    def _scripttype_childscript(self) -> int:
        return self._const("scripttype_childscript", 1)

    def _scriptattribute_enabled(self) -> int:
        return self._const("scriptattribute_enabled", 1)

    def get_pose2d_world(self, object_handle: int) -> Pose2D:
        self._ensure_ready()
        try:
            pos = self.simulation.getObjectPosition(object_handle, self._handle_world())
            qx, qy, qz, qw = self.simulation.getObjectQuaternion(object_handle, self._handle_world())
            _, _, yaw = quaternion_to_euler(qx, qy, qz, qw)
            return Pose2D(x=pos[0], y=pos[1], theta=yaw)
        except Exception as e:
            raise RuntimeError(f"[ERRO] Falha ao obter pose do objeto '{object_handle}': {e}")

    def set_pose2d_world(self, object_handle: int, pose: Union[Pose2D, tuple, list]):
        self._ensure_ready()
        try:
            if isinstance(pose, Pose2D):
                pos_x, pos_y, yaw = pose.x, pose.y, pose.theta
            elif isinstance(pose, (tuple, list)) and len(pose) == 3:
                pos_x, pos_y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
            else:
                raise ValueError("[ERRO] set_pose2d_world espera Pose2D ou (x,y,theta).")

            current_z = self.simulation.getObjectPosition(object_handle, self._handle_world())[2]
            qx, qy, qz, qw = self.simulation.getObjectQuaternion(object_handle, self._handle_world())
            roll, pitch, _ = quaternion_to_euler(qx, qy, qz, qw)
            new_q = euler_to_quaternion(roll, pitch, yaw)

            self.simulation.setObjectPosition(object_handle, [pos_x, pos_y, current_z], self._handle_world())
            self.simulation.setObjectQuaternion(object_handle, list(new_q), self._handle_world())
        except Exception as e:
            raise RuntimeError(f"[ERRO] Falha ao definir pose do objeto '{object_handle}': {e}")

    def _ensure_ready(self):
        if self.client is None or self.simulation is None:
            raise RuntimeError("[ERRO] Sessão não está conectada. Use connect() ou with CoppeliaSimSession().")


def _normalize(s: str) -> str:
    return re.sub(r'[^a-z0-9]+', '', (s or '').lower())

def list_scene(session: CoppeliaSimSession) -> List[str]:
    sim = session.simulation
    handles: List[int] = []
    try:
        h = sim.getObjects(-1, 0)
        if isinstance(h, list): handles += h
    except Exception: pass
    try:
        root = sim.getObject('/')
        h2 = sim.getObjectsInTree(root, -1, 0)
        if isinstance(h2, list): handles += h2
    except Exception: pass
    for t in range(0, 100):
        try:
            ht = sim.getObjects(t, 0)
            if isinstance(ht, list): handles += ht
        except Exception:
            continue
    seen, out = set(), []
    for h in handles:
        if h in seen: continue
        seen.add(h)
        try:
            out.append(sim.getObjectAlias(h, 1))
        except Exception:
            pass
    return sorted(out)

def find_epuck(session: CoppeliaSimSession) -> int:
    sim = session.simulation
    for p in EPUCK_PATH_HINTS:
        try:
            return sim.getObject(p)
        except Exception:
            pass
    for alias in list_scene(session):
        if 'epuck' in _normalize(alias):
            try:
                return sim.getObject(alias)
            except Exception:
                short = alias.split('/')[-1]
                try:
                    return sim.getObject('/' + short)
                except Exception:
                    continue
    raise RuntimeError("e-puck não encontrado. Verifique o path no Scene Hierarchy e adicione em EPUCK_PATH_HINTS.")

def disable_child_script(session: CoppeliaSimSession, model_handle: int) -> None:
    sim = session.simulation
    try:
        scr_type = session._scripttype_childscript()
        attr_enabled = session._scriptattribute_enabled()
        scr = sim.getScript(scr_type, model_handle)
        sim.setScriptAttribute(scr, attr_enabled, False)
        print("Child script do e-puck desabilitado.")
    except Exception:
        pass

def find_wheel_joints(session: CoppeliaSimSession, epuck_handle: int) -> Tuple[int, int]:
    sim = session.simulation
    t_joint = session._object_joint_type()
    joints = sim.getObjectsInTree(epuck_handle, t_joint, 0)
    left = right = None

    for j in joints:
        name = (sim.getObjectAlias(j, 0) or '').lower()
        if re.search(r'\b(left|lwheel|l_motor|l-?joint|leftmotor|motorleft)\b', name):
            left = j
        elif re.search(r'\b(right|rwheel|r_motor|r-?joint|rightmotor|motorright)\b', name):
            right = j

    if left is None or right is None:
        revolute_sub = session._joint_revolute_subtype()
        revolutes = [j for j in joints if sim.getJointType(j) == revolute_sub]
        if len(revolutes) >= 2:
            if left is None: left = revolutes[0]
            if right is None:
                right = next((r for r in revolutes if r != left), None)

    if left is None or right is None:
        raise RuntimeError("Juntas das rodas não identificadas. Renomeie (leftMotor/rightMotor) ou ajuste a regex.")
    return left, right

def set_wheel_speeds(session: CoppeliaSimSession, left_joint: int, right_joint: int,
                     v_left: float, v_right: float) -> None:
    sim = session.simulation
    sim.setJointTargetVelocity(left_joint, float(v_left))
    sim.setJointTargetVelocity(right_joint, float(v_right))

def main():
    with CoppeliaSimSession(host="127.0.0.1", port=23000, auto_connect=True) as sess:
        sess.start()
        sess.wait_until_running(5.0)

        epuck = find_epuck(sess)
        print("e-puck:", sess.simulation.getObjectAlias(epuck, 1))

        disable_child_script(sess, epuck)

        left, right = find_wheel_joints(sess, epuck)
        print("rodas:", sess.simulation.getObjectAlias(left, 1), "|", sess.simulation.getObjectAlias(right, 1))

        try:
            set_wheel_speeds(sess, left, right, 5.0, 5.0)
            for _ in range(150): sess.step(); time.sleep(0.01)

            set_wheel_speeds(sess, left, right, -4.0, 4.0)
            for _ in range(120): sess.step(); time.sleep(0.01)

            set_wheel_speeds(sess, left, right, 0.0, 0.0)
            for _ in range(10): sess.step(); time.sleep(0.01)
        finally:
            sess.stop()

if __name__ == "__main__":
    main()
