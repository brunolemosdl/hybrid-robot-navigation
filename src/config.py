from dotenv import load_dotenv
import os

load_dotenv()

COPPELIA_HOST = os.getenv("COPPELIA_HOST")
COPPELIA_PORT = int(os.getenv("COPPELIA_PORT", 23000))
EPUCK_PATH_HINTS = os.getenv("EPUCK_PATH_HINTS")
