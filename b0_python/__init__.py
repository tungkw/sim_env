import os
import sys
current_os_path = os.path.abspath(".")
os.chdir(os.path.dirname(__file__))
sys.path.append(os.path.dirname(__file__))
from . import b0RemoteApi
os.chdir(current_os_path)
sys.path.pop()

