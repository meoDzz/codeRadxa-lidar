import glob
import logging

class DeviceScanner:
    def __init__(self, pattern='/dev/ttyACM*'):
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger("Scanner")
    def scan(self):
        """Quét các cổng nối tiếp khớp với mẫu đã cho"""
        
        pattern_usbPorts = '/dev/ttyUSB*'
        pattern_acmPorts = '/dev/ttyACM*'

        found_devices = { "lidar": None, "esp32": None }
        #1 find lidar sensor
        usb_ports = glob.glob(pattern_usbPorts)
        print(usb_ports)
        if len(usb_ports) > 0:
            found_devices["lidar"] = usb_ports[0]
            self.logger.info(f"Tìm thấy LIDAR tại {usb_ports[0]}")
        else:
            self.logger.warning("Không tìm thấy LIDAR nào!")
        
        #2 find esp32
        acm_ports = glob.glob(pattern_acmPorts)
        if len(acm_ports) > 0:
            found_devices["esp32"] = acm_ports[0]
            self.logger.info(f"Tìm thấy ESP32 tại {acm_ports[0]}")
        else:
            self.logger.warning("Không tìm thấy ESP32 nào!")
        
        return found_devices

if __name__ == "__main__":
    scanner = DeviceScanner()
    devices = scanner.scan()
    print(devices)