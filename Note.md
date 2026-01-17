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


set rules cho cổng kết nối

Lidar A1M8 cổng USB physical 1-1.3 (bên trái trên cùng)
Vi điều khiển cổng USB physical 1-1.2 (ngay phía dưới)

Lệnh kiểm tra
udevadm info -a -n /dev/ttyUSB0 | grep KERNELS 
tìm cái dòng số kiểu 1-1.4 hay 1-1.3 (chọn dòng thứ 3 trong danh sách cho chắc).

chỗ lưu file config cổng
sudo nano /etc/udev/rules.d/99-robot-names.rules

    # 1. Cổng dành cho LIDAR (Bắt buộc là Lidar)
    SUBSYSTEM=="tty", KERNELS=="1-1.3", SYMLINK+="lidar", MODE="0666"

    # 2. Cổng dành cho VI ĐIỀU KHIỂN (USB hay ACM đều nhận hết)
    # Chỉ cần nó là TTY và cắm đúng lỗ 1-1.4
    SUBSYSTEM=="tty", KERNELS=="1-1.2", SYMLINK+="esp", MODE="0666"


Nạp luật
sudo udevadm control --reload-rules && sudo udevadm trigger
