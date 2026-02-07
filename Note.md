###Setup lidar

ls -l /dev/ttyUSB*
```
sudo chmod 666 /dev/ttyUSB0
```
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






# 🤖 Hướng dẫn Cấu hình Phần cứng (Lidar & ESP32)

Tài liệu này hướng dẫn thiết lập quyền truy cập cổng USB và tạo định danh cố định (Symlink) cho Lidar và Vi điều khiển (ESP32) trên hệ thống ROS 2.

## 1. Kiểm tra & Cấp quyền thủ công (Tạm thời)
*Dùng để test nhanh, quyền sẽ mất sau khi khởi động lại.*

### Cấu hình Lidar (A1M8)
```bash
# Kiểm tra thiết bị đang kết nối
ls -l /dev/ttyUSB*

# Cấp quyền đọc/ghi
sudo chmod 666 /dev/ttyUSB0

# Chạy thử Lidar trên ROS 2
cd ros2_ws
source install/setup.bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/ttyUSB0 serial_baudrate:=115200
```

### Cấu hình Vi điều khiển (ESP32)
```bash
# Kiểm tra thiết bị (thường là ACM hoặc USB)
ls -l /dev/ttyACM*

# Cấp quyền đọc/ghi
sudo chmod 666 /dev/ttyACM*
```

### Cài đặt thư viện Python cần thiết
```bash
sudo apt install python3-serial
```

---

## 2. Thiết lập cố định cổng (Udev Rules) - Khuyên dùng
Phương pháp này giúp hệ thống tự động nhận diện thiết bị dựa trên **Lỗ cắm vật lý (Physical Port)**, tránh tình trạng bị nhảy tên cổng (USB0 thành USB1) khi khởi động lại.

### Bước 1: Xác định địa chỉ vật lý (KERNELS)
Quy hoạch lỗ cắm:
* **Lidar A1M8:** Cắm cổng USB bên trái trên cùng.
* **Vi điều khiển:** Cắm cổng USB ngay phía dưới.

Lệnh kiểm tra để lấy mã `KERNELS`:
```bash
# Thay ttyUSB0 bằng cổng thực tế đang cắm để kiểm tra
udevadm info -a -n /dev/ttyUSB0 | grep KERNELS
```
> **Mẹo:** Tìm dòng số có dạng `1-1.3` hoặc `1-1.2` (thường nằm ở dòng thứ 3 trong danh sách kết quả).

### Bước 2: Tạo file luật (Rules)
Mở trình soạn thảo Nano:
```bash
sudo nano /etc/udev/rules.d/99-robot-names.rules
```

Dán nội dung sau vào file (sử dụng chuột phải để paste):

```bash
# 1. Cổng dành cho LIDAR (Bắt buộc là Lidar)
# Physical Port: 1-1.3 (Trái trên)
SUBSYSTEM=="tty", KERNELS=="1-1.3", SYMLINK+="lidar", MODE="0666"

# 2. Cổng dành cho VI ĐIỀU KHIỂN (USB hay ACM đều nhận hết)
# Chỉ cần nó là TTY và cắm đúng lỗ 1-1.2 (Dưới Lidar)
SUBSYSTEM=="tty", KERNELS=="1-1.2", SYMLINK+="esp", MODE="0666"
```

> **Lưu ý trong Nano:**
> * Lưu file: `Ctrl` + `O` -> `Enter`
> * Thoát: `Ctrl` + `X`

### Bước 3: Áp dụng cấu hình
Chạy lệnh sau để hệ thống nạp lại luật mới mà không cần khởi động lại máy:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Bước 4: Kiểm tra kết quả
```bash
ls -l /dev/lidar /dev/esp
```
*Nếu thành công, hệ thống sẽ hiện ra 2 thiết bị trỏ về ttyUSBx tương ứng.*

---

## 3. Cập nhật lệnh chạy ROS 2
Sau khi đã set rules thành công, câu lệnh chạy Lidar sẽ thay đổi cổng như sau:

Lidar A1M8
```bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/lidar serial_baudrate:=115200
```

Lidar C1M1_R2
```
ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/lidar serial_baudrate:=460800
```
Trong code Python điều khiển ESP32, khai báo cổng kết nối là:
```python
self.driver = SerialDriver(port='/dev/esp')
```


# 
# Start robot and sensor

## 1. Start the ros2 environment and run lidar sensor
```
cd ros2_ws
source install/setup.bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/lidar serial_baudrate:=115200
```
## 2. Run the robot and transfer data to microcontroller
```
cd ros2_ws/scripts
python3 robot_main.py
```

# Run rviz2
```
ros2 run rviz2 rviz2
```


# Data Flow System
## Sơ đồ hệ thống Robot (CAN Bus & ROS2)

![Sơ đồ xe tự hành](./Diagram_System/dataFlow.drawio.svg)

# NOTE
## ADD new commit
```
git add .
```
```
git commit -m " nội dung ghi vào đây"
```
```
git push
```

## Update file 
```
git status
```

```
git fetch
```

```
git pull
```
