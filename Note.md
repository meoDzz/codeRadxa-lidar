Setup lidar

ls -l /dev/ttyUSB*

sudo chmod 666 /dev/ttyUSB0

cd ros2_ws
source install/setup.bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/ttyUSB0 serial_baudrate:=115200
