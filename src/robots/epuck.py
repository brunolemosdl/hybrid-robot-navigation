import re
from typing import Tuple, List
from src.core.session import CoppeliaSimSession

def _normalize(s: str) -> str:
    return re.sub(r'[^a-z0-9]+', '', (s or '').lower())

def find_epuck(session: CoppeliaSimSession) -> int:
    sim = session.simulation
    for p in EPUCK_PATH_HINTS:
        try:
            return sim.getObject(p)
        except Exception:
            pass
    # fallback simples: varre por tipos 0..99 e acha path com 'epuck'
    handles = []
    for t in range(100):
        try:
            hs = sim.getObjects(t, 0)
            if isinstance(hs, list): handles += hs
        except Exception: pass
    for h in handles:
        alias = (sim.getObjectAlias(h, 1) or '')
        if 'epuck' in _normalize(alias):
            return sim.getObject(alias)
    raise RuntimeError("e-puck não encontrado. Ajuste EPUCK_PATH_HINTS.")

def list_children(session: CoppeliaSimSession, model: int) -> List[str]:
    sim = session.simulation
    out = []
    for h in sim.getObjectsInTree(model, -1, 0):
        out.append(sim.getObjectAlias(h, 1))
    return out

def disable_child_script(session: CoppeliaSimSession, model_handle: int) -> None:
    sim = session.simulation
    try:
        scr = sim.getScript(1, model_handle)          # 1 = child script
        sim.setScriptAttribute(scr, 1, False)         # 1 = enabled
    except Exception:
        pass

def find_wheel_joints(session: CoppeliaSimSession, model: int) -> Tuple[int, int]:
    sim = session.simulation
    joints = sim.getObjectsInTree(model, 2, 0)       # 2 = joint
    left = right = None
    for j in joints:
        name = (sim.getObjectAlias(j, 0) or '').lower()
        if re.search(r'(left|lwheel|leftmotor|motorleft)', name): left = j
        if re.search(r'(right|rwheel|rightmotor|motorright)', name): right = j
    if left is None or right is None:
        revolutes = [j for j in joints if sim.getJointType(j) == 10]  # 10 = revolute
        if len(revolutes) >= 2:
            left = revolutes[0] if left is None else left
            right = next((r for r in revolutes if r != left), revolutes[1]) if right is None else right
    if left is None or right is None:
        raise RuntimeError("Juntas das rodas não identificadas.")
    return left, right

def set_wheel_speeds(session: CoppeliaSimSession, left: int, right: int, wl: float, wr: float) -> None:
    sim = session.simulation
    sim.setJointTargetVelocity(left, float(wl))
    sim.setJointTargetVelocity(right, float(wr))
