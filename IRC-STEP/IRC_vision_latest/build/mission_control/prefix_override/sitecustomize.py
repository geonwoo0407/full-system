import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jet/IRC/IRC-STEP/IRC_vision_latest/install/mission_control'
