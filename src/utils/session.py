from services.coppelia import CoppeliaSimSession

_COPPELIA_SESSION = None


def get_coppelia_session(host="localhost", port=23000):
    global _COPPELIA_SESSION
    if _COPPELIA_SESSION is None:
        _COPPELIA_SESSION = CoppeliaSimSession(host=host, port=port)
    return _COPPELIA_SESSION


def reset_coppelia_session():
    global _COPPELIA_SESSION
    if _COPPELIA_SESSION:
        _COPPELIA_SESSION.close()
    _COPPELIA_SESSION = None
