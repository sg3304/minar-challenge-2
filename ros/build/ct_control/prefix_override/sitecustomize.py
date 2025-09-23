import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/samuele/dev/minar/minar-challenge-2/ros/install/ct_control'
