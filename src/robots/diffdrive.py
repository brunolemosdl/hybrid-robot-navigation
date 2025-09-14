def diff_drive(v: float, w: float, R: float = 0.0205, L: float = 0.052):
    v_l = v - w * L
    v_r = v + w * L
    return v_l / R, v_r / R


def inv_diff_drive(omega_l: float, omega_r: float, R: float = 0.0205, L: float = 0.052):
    v_l, v_r = omega_l * R, omega_r * R
    v = (v_l + v_r) / 2
    w = (v_r - v_l) / (2 * L)
    return v, w
