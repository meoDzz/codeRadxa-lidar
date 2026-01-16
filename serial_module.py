import serial
import time
import logging

class SerialDriver:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.logger = logging.getLogger("SerialDriver")
        self.connect()

    def connect(self):
        """Mở kết nối Serial an toàn"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
            self.logger.info(f"Đã kết nối ESP32 tại {self.port}")
            time.sleep(2) # Chờ ESP32 khởi động lại sau khi kết nối
        except serial.SerialException as e:
            self.logger.error(f"Không thể kết nối ESP32: {e}")

    def send_velocity(self, linear_x, angular_z):
        """
        Gửi lệnh vận tốc xuống ESP32
        Format: VEL:linear:angular\n
        """
        if self.serial_conn and self.serial_conn.is_open:
            # Tạo chuỗi lệnh
            cmd = f"{linear_x:.2f},{angular_z:.2f}\n"
            
            try:
                # Gửi dạng bytes (encode utf-8)
                self.serial_conn.write(cmd.encode('utf-8'))
                self.logger.debug(f"Đã gửi: {cmd.strip()}")
            except Exception as e:
                self.logger.error(f"Lỗi gửi dữ liệu: {e}")
        else:
            self.logger.warning("Cổng Serial chưa mở, đang thử kết nối lại...")
            self.connect()

    def close(self):
        if self.serial_conn:
            self.serial_conn.close()

# --- CÁCH DÙNG THỬ (Test độc lập) ---
if __name__ == "__main__":
    # Lưu ý: Trên Radxa, ESP32 thường nhận là /dev/ttyACM0 hoặc /dev/ttyUSB0
    driver = SerialDriver(port='/dev/ttyACM0') 
    
    try:
        while True:
            print("Gửi lệnh chạy thẳng...")
            driver.send_velocity(0.5, 0.0)
            time.sleep(1)
            
            print("Gửi lệnh rẽ trái...")
            driver.send_velocity(0.2, 0.5)
            time.sleep(1)
            
    except KeyboardInterrupt:
        driver.close()