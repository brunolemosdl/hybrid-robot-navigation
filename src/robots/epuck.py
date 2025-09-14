import re
from typing import Optional, Tuple, List
from src.core.session import CoppeliaSimSession
from src.config import EPUCK_PATH_HINTS


def _normalize(s: str) -> str:
    return re.sub(r"[^a-z0-9]+", "", (s or "").lower())


def list_scene(session: CoppeliaSimSession) -> List[str]:
    sim = session.simulation
    handles: List[int] = []
    try:
        h = sim.getObjects(-1, 0)
        if isinstance(h, list):
            handles += h
    except Exception:
        pass
    try:
        root = sim.getObject("/")
        h2 = sim.getObjectsInTree(root, -1, 0)
        if isinstance(h2, list):
            handles += h2
    except Exception:
        pass
    for t in range(0, 100):
        try:
            ht = sim.getObjects(t, 0)
            if isinstance(ht, list):
                handles += ht
        except Exception:
            continue
    seen, out = set(), []
    for h in handles:
        if h in seen:
            continue
        seen.add(h)
        try:
            out.append(sim.getObjectAlias(h, 1))
        except Exception:
            pass
    return sorted(out)


def find_epuck(session: CoppeliaSimSession, hints: Optional[List[str]] = None) -> int:
    """
    Tenta resolver o handle do e-puck:
    1) tenta hints explícitos (se dados) + '/ePuck' como fallback imediato
    2) se falhar, varre a cena procurando qualquer alias com 'epuck'
    """
    sim = session.simulation
    # prioriza paths explícitos (se vieram) e SEMPRE tenta '/ePuck'
    tried = set()
    ordered_hints = []
    if hints:
        ordered_hints.extend(hints)
    ordered_hints.append("/ePuck")

    for p in ordered_hints:
        if not p or p in tried:
            continue
        tried.add(p)
        try:
            return sim.getObject(p)
        except Exception:
            pass

    for alias in list_scene(session):
        if "epuck" in _normalize(alias):
            try:
                return sim.getObject(alias)
            except Exception:
                short = alias.split("/")[-1]
                try:
                    return sim.getObject("/" + short)
                except Exception:
                    continue

    raise RuntimeError(
        "e-puck não encontrado. Tente find_epuck(sess, hints=['/ePuck']) ou ajuste os paths."
    )

def _get_flag(sim, name: str, default: int) -> int:
    # obtém constante se existir; senão usa fallback numérico
    return getattr(sim, name, default)

def disable_child_script(session, model_handle: int, verbose: bool = True) -> None:
    """
    Desabilita o child script do modelo sem matar o add-on:
    1) tenta API nova: getScriptProperty/setScriptProperty limpando o bit 'enabled';
    2) fallback: setScriptAttribute(..., enabled, False).
    """
    sim = session.simulation
    try:
        # 1 = child script (fallback numérico para builds sem a constante)
        scripttype_child = _get_flag(sim, "scripttype_childscript", 1)
        scr = sim.getScript(scripttype_child, model_handle)
    except Exception:
        if verbose: print("[warn] ePuck não possui child script associado.")
        return

    # --- caminho novo: properties ---
    try:
        props = sim.getScriptProperty(scr)  # algumas builds retornam int; outras retornam (props, objHandle)
        if isinstance(props, (list, tuple)):
            props = props[0]
        enabled_bit = _get_flag(sim, "scriptproperty_enabled", 1)
        new_props = int(props) & ~int(enabled_bit)
        sim.setScriptProperty(scr, new_props)
        if verbose: print("[ok] Child script desabilitado via properties.")
        return
    except Exception as e:
        if verbose: print(f"[info] API 'properties' indisponível ({e}); tentando atributo legado...")

    # --- fallback legado: attribute ---
    try:
        enabled_attr = _get_flag(sim, "scriptattribute_enabled", 1)
        sim.setScriptAttribute(scr, enabled_attr, False)
        if verbose: print("[ok] Child script desabilitado via attribute (LEGACY).")
    except Exception as e:
        if verbose: print(f"[erro] Não foi possível desabilitar child script: {e}")


def enable_child_script(session, model_handle: int, verbose: bool = True) -> None:
    """Reverte o disable (útil p/ testes)."""
    sim = session.simulation
    try:
        scripttype_child = _get_flag(sim, "scripttype_childscript", 1)
        scr = sim.getScript(scripttype_child, model_handle)
    except Exception:
        if verbose: print("[warn] Sem child script p/ habilitar.")
        return
    try:
        props = sim.getScriptProperty(scr)
        if isinstance(props, (list, tuple)):
            props = props[0]
        enabled_bit = _get_flag(sim, "scriptproperty_enabled", 1)
        sim.setScriptProperty(scr, int(props) | int(enabled_bit))
        if verbose: print("[ok] Child script habilitado via properties.")
        return
    except Exception:
        pass
    try:
        enabled_attr = _get_flag(sim, "scriptattribute_enabled", 1)
        sim.setScriptAttribute(scr, enabled_attr, True)
        if verbose: print("[ok] Child script habilitado via attribute (LEGACY).")
    except Exception as e:
        if verbose: print(f"[erro] Não foi possível habilitar child script: {e}")


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


def set_wheel_speeds(
    session: CoppeliaSimSession, left: int, right: int, wl: float, wr: float
) -> None:
    sim = session.simulation
    sim.setJointTargetVelocity(left, float(wl))
    sim.setJointTargetVelocity(right, float(wr))
