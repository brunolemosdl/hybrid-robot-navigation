from typing import Optional, Union
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from .geometry import Pose2D
from src.core.geometry import quaternion_to_euler, euler_to_quaternion


class CoppeliaSimSession:
    def __init__(
        self, host: str = "localhost", port: int = 23000, auto_connect: bool = True
    ):
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
        return (
            self.simulation.getSimulationState() != self.simulation.simulation_stopped
        )

    def wait_until_running(self, timeout_seconds: float = 5.0):
        self._ensure_ready()
        t0 = time.time()
        while not self.is_running():
            if time.time() - t0 > timeout_seconds:
                raise TimeoutError(
                    "[ERRO] A simulação não iniciou dentro do tempo limite."
                )
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
            qx, qy, qz, qw = self.simulation.getObjectQuaternion(
                object_handle, self._handle_world()
            )
            _, _, yaw = quaternion_to_euler(qx, qy, qz, qw)
            return Pose2D(x=pos[0], y=pos[1], theta=yaw)
        except Exception as e:
            raise RuntimeError(
                f"[ERRO] Falha ao obter pose do objeto '{object_handle}': {e}"
            )

    def set_pose2d_world(self, object_handle: int, pose: Union[Pose2D, tuple, list]):
        self._ensure_ready()
        try:
            if isinstance(pose, Pose2D):
                pos_x, pos_y, yaw = pose.x, pose.y, pose.theta
            elif isinstance(pose, (tuple, list)) and len(pose) == 3:
                pos_x, pos_y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
            else:
                raise ValueError(
                    "[ERRO] set_pose2d_world espera Pose2D ou (x,y,theta)."
                )

            current_z = self.simulation.getObjectPosition(
                object_handle, self._handle_world()
            )[2]
            qx, qy, qz, qw = self.simulation.getObjectQuaternion(
                object_handle, self._handle_world()
            )
            roll, pitch, _ = quaternion_to_euler(qx, qy, qz, qw)
            new_q = euler_to_quaternion(roll, pitch, yaw)

            self.simulation.setObjectPosition(
                object_handle, [pos_x, pos_y, current_z], self._handle_world()
            )
            self.simulation.setObjectQuaternion(
                object_handle, list(new_q), self._handle_world()
            )
        except Exception as e:
            raise RuntimeError(
                f"[ERRO] Falha ao definir pose do objeto '{object_handle}': {e}"
            )

    def _ensure_ready(self):
        if self.client is None or self.simulation is None:
            raise RuntimeError(
                "[ERRO] Sessão não está conectada. Use connect() ou with CoppeliaSimSession()."
            )
