import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lachlanselleck/ros2_ws/src/RS2_SNL/rs2_snl_pkg/install/rs2_snl_pkg'
