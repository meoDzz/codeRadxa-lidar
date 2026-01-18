import os
import logging

class DeviceScanner:
    def __init__(self):
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger("Scanner")

    def scan(self):
        """
        Kiểm tra xem các cổng đã Set cứng (symlink) có tồn tại không.
        Không cần quan tâm nó là ttyUSB hay ttyACM gốc.
        """
        found_devices = {
            "lidar": None,
            "esp32": None
        }

        # 1. KIỂM TRA LIDAR (Đã định danh là /dev/lidar)
        if os.path.exists('/dev/lidar'):
            found_devices["lidar"] = '/dev/lidar'
            self.logger.info(f"✅ Đã thấy LIDAR tại: /dev/lidar")
            # Mẹo: Anh có thể dùng os.path.realpath('/dev/lidar') nếu muốn biết cổng gốc của nó
        else:
            self.logger.error("❌ Lỗi: Không tìm thấy thiết bị '/dev/lidar'. Kiểm tra dây cắm!")

        # 2. KIỂM TRA ESP32 (Đã định danh là /dev/esp)
        # Bất kể là ACM hay USB, nếu rule chạy đúng, file này sẽ xuất hiện
        if os.path.exists('/dev/esp'):
            found_devices["esp32"] = '/dev/esp'
            self.logger.info(f"✅ Đã thấy ESP32 tại: /dev/esp")
        else:
            self.logger.error("❌ Lỗi: Không tìm thấy thiết bị '/dev/esp'.")
            
            # --- PHƯƠNG ÁN DỰ PHÒNG (FALLBACK) ---
            # Nếu lỡ quên set rule cho ESP, ta mới đi mò thủ công
            self.logger.warning("⚠️ Đang thử tìm thủ công trong ttyACM/ttyUSB...")
            import glob
            temp_list = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
            # Lọc bỏ cổng của Lidar ra (nếu Lidar đang chiếm ttyUSB0)
            # Logic này chỉ là dự phòng, không khuyến khích dùng lâu dài
            if len(temp_list) > 0:
                 self.logger.info(f"👉 Tìm thấy cổng tiềm năng: {temp_list}")
        
        return found_devices

if __name__ == "__main__":
    scanner = DeviceScanner()
    devices = scanner.scan()
    print("Kết quả:", devices)