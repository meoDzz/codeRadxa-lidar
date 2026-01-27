import serial
import time

# Cấu hình cổng kết nối
# Thay '/dev/ttyUSB0' bằng cổng bạn tìm thấy ở bước 2
# 115200 là baudrate mặc định thông dụng của ESP

def connectSerial():
    while True:
        try:
            ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            ser.flush()
            print("Kết nối thành công với Safety Sensor System!")
            return ser
        except serial.SerialException as e:
            print(f"Lỗi kết nối: {e}. Đang thử lại Safety Sensor System...")
            time.sleep(0.1)

ser = connectSerial()
ser.flush()

print("Đang chờ tín hiệu từ ESP...")

try:
    while True:
        try:
            if ser.in_waiting > 0:
                # Đọc dòng dữ liệu và giải mã
                line = ser.readline().decode('utf-8').rstrip()
                if line:
                    print(f"Nhận được: {line}")
        except (serial.SerialException, OSError) as e:
            print(f"Lỗi kết nối: {e}. Đang thử kết nối lại Safety Sensor System...")
            ser.close()
            ser = connectSerial()
except KeyboardInterrupt:
    print("\nĐã dừng chương trình bởi người dùng.")
finally:
    if ser and ser.is_open:
        ser.close()