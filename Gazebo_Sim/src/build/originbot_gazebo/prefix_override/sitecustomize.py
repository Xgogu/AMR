import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mgshawn/Working_WS/AMR/Gazebo_Sim/src/install/originbot_gazebo'
