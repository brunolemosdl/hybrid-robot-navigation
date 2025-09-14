import time
from src.core import CoppeliaSimSession
from src.robots.epuck import (
    find_epuck,
    find_wheel_joints,
    set_wheel_speeds,
    disable_child_script,
)
from src.config import COPPELIA_HOST, COPPELIA_PORT


def main():
    with CoppeliaSimSession(host=COPPELIA_HOST, port=COPPELIA_PORT) as sess:
        sess.start()
        sess.wait_until_running()
        ep = find_epuck(sess, hints=["/ePuck", "/ePuck#0"])
        disable_child_script(sess, ep)
        l, r = find_wheel_joints(sess, ep)
        try:
            set_wheel_speeds(sess, l, r, 5.0, 5.0)
            for _ in range(150):
                sess.step()
                time.sleep(0.01)
        finally:
            set_wheel_speeds(sess, l, r, 0.0, 0.0)
            for _ in range(5):
                sess.step()
            sess.stop()


if __name__ == "__main__":
    main()
