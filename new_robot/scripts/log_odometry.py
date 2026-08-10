import csv

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdometryLogger(Node):
    def __init__(self):
        super().__init__('odometry_logger')
        self.declare_parameter('output_path', 'odometry_log.csv')
        output_path = self.get_parameter('output_path').value
        self.file = open(output_path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'stamp_sec', 'stamp_nanosec',
            'pos_x', 'pos_y', 'pos_z',
            'quat_x', 'quat_y', 'quat_z', 'quat_w',
            'lin_vel_x', 'lin_vel_y', 'lin_vel_z',
            'ang_vel_x', 'ang_vel_y', 'ang_vel_z',
        ])
        self.subscription = self.create_subscription(
            Odometry, '/model/new_robot/odometry', self.callback, 50)
        self.get_logger().info(f'Logging odometry to {output_path}')

    def callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        lv = msg.twist.twist.linear
        av = msg.twist.twist.angular
        self.writer.writerow([
            msg.header.stamp.sec, msg.header.stamp.nanosec,
            p.x, p.y, p.z,
            q.x, q.y, q.z, q.w,
            lv.x, lv.y, lv.z,
            av.x, av.y, av.z,
        ])
        self.file.flush()

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = OdometryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()