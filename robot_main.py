
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import time
from serial_module import SerialDriver

class LidarReader(Node):
    def __init__(self):
        super().__init__('lidar_reader_node')
        # Đăng ký nhận tin từ topic '/scan'
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10) # 10 là kích thước hàng chờ (QoS)
        # self.last_has_obstacle = None
        self.driver = SerialDriver(port='/dev/esp')


    def listener_callback(self, msg):
        # msg.ranges là một danh sách chứa hàng trăm con số khoảng cách
        # Lấy phần tử ở giữa danh sách (thường là góc trước mặt robot)
        center_index = int(len(msg.ranges) / 2)
        distance = msg.ranges[center_index]

        # In ra màn hình
        # print(msg)
        # print("Min",min(msg.ranges))

        if distance == float('inf'):
            print("Phía trước thoáng (Ngoài tầm quét)")
        else:
            # Hiển thị khoảng cách (làm tròn 2 số lẻ)
            print(f"Vật cản phía trước cách: {distance:.2f} mét")

        # print(msg.angle_min)
        # print(msg.angle_max)
        # print(msg.angle_increment)
        print("----")
        processed_ranges =[]
        for i, d in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            # print(f" Góc: {angle:.2f} rad, Góc: {angle * 180 / math.pi:.2f} degree , Khoảng cách: {d:.2f} m")
            processed_ranges.append((angle, d))
        # print(processed_ranges)

        if processed_ranges:  # Kiểm tra list có dữ liệu không để tránh lỗi crash
            # Tìm tuple có d nhỏ nhất
            closest_point = min(processed_ranges, key=lambda x: x[1])

            # Tách ra để sử dụng
            min_angle = closest_point[0]
            min_dist  = closest_point[1]
            # self.driver.send_velocity(f"{min_dist:.2f}, {math.degrees(min_angle):.1f}")
            #self.driver.send_velocity(min_dist,math.degrees(min_angle))
            #time.sleep(2)
            #print(f"{min_dist:.2f}, {math.degrees(min_angle):.1f}")
            #current_has_obstacle = (0.15 <= min_dist <= 0.8)
            #if current_has_obstacle != self.last_has_obstacle:
            #    print(f"{min_dist:.2f}, {math.degrees(min_angle):.1f}")
            #   self.driver.send_velocity(min_dist,math.degrees(min_angle))
            #    self.last_has_obstacle = current_has_obstacle
            if 0.05 < min_dist < 0.8:
                self.driver.send_velocity(min_dist,math.degrees(min_angle))
                print(f"Vật gần nhất cách {min_dist:.2f}m ở góc {math.degrees(min_angle):.1f} độ")
        else:
            print("Không tìm thấy vật cản nào!")

def main(args=None):
    rclpy.init(args=args)
    node = LidarReader()
    try:
        rclpy.spin(node) # Giữ cho node chạy liên tục
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
