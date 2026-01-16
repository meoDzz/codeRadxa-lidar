Setup lidar

ls -l /dev/ttyUSB*

sudo chmod 666 /dev/ttyUSB0

cd ros2_ws
source install/setup.bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/ttyUSB0 serial_baudrate:=115200



ESP32

ls -l /dev/ttyACM*
sudo chmod 666 /dev/ttyACM*

cài thư viện pyserial:
sudo apt install python3-serial
