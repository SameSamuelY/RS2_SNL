import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/boinggboi/projects/RS2_SNL/src/install/gui_control'
