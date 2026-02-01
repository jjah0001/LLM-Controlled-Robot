import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jeffreyye/Documents/LLM-Controlled-Robot/install/motor_driver'
