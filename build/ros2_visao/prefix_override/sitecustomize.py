import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
<<<<<<< HEAD
    sys.prefix = sys.exec_prefix = '/home/droid/WORK2-2026/install/ros2_visao'
=======
    sys.prefix = sys.exec_prefix = '/home/daniel/unb/droid/WORK2-2026/install/ros2_visao'
>>>>>>> garra_nao_simultanea
