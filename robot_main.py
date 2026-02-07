import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
from serial_module import SerialDriver

class LidarReader(Node):
    def __init__(self):
        super().__init__('lidar_reader_node')
        # Đăng ký nhận tin từ topic '/scan'
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        
        # Khởi tạo driver để gửi lệnh xuống VĐK qua cổng /dev/esp
        self.driver = SerialDriver(port='/dev/esp')
        self.get_logger().info("Lidar Reader Node has started.")

    def listener_callback(self, msg):
        # 1. Tạo danh sách points [góc, khoảng cách] và lọc 'vùng chết' 66-73 độ ngay lập tức
        # Loại bỏ luôn các điểm có khoảng cách <= 0 (nhiễu hoặc lỗi tia)
        points = []
        for i, dist in enumerate(msg.ranges):
            # Kiểm tra khoảng cách hợp lệ trước khi tính toán góc để tiết kiệm CPU
            if dist > 0.02 and dist != float('inf'):
                angle = msg.angle_min + i * msg.angle_increment
                angle_deg = math.degrees(angle)
                points.append((angle, dist))
                
        # 2. Xử lý sau khi đã lọc
        if points:
            # Tìm vật cản gần nhất trong số các điểm hợp lệ
            closest_point = min(points, key=lambda x: x[1])
            min_angle = closest_point[0]
            min_dist  = closest_point[1]
            min_angle_deg = math.degrees(min_angle)

            # 3. Logic an toàn: Chỉ gửi lệnh nếu vật cản nằm trong tầm 0.05m - 0.8m
            if 0.01 < min_dist < 0.2:
                # Gửi dữ liệu xuống VĐK1 (Motor) qua Serial/USB
                # Lidar C1
                if 0 < min_angle_deg < 180:
                    min_angle_deg = min_angle_deg - 180
                else:
                    min_angle_deg = min_angle_deg + 180
                
                if not (80 <= min_angle_deg <= 120) and not (-120 <= min_angle_deg <= -80):
                    self.driver.send_velocity(min_dist, min_angle_deg)    
                    print(f"--- VẬT CẢN: {min_dist:.2f}m tại {min_angle_deg:.1f} độ")
                else:
                    print("Vật cản nằm trong vùng chết, không gửi lệnh.")
            else:
                # Nếu vật ở xa hơn 0.8m, có thể in ra để debug (tùy chọn)
                print("Vùng quét an toàn không có vật cản.")
                pass 
        else:
            print("Vùng quét an toàn không có vật cản.")

def main(args=None):
    rclpy.init(args=args)
    node = LidarReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nNode đang dừng...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
