#!/usr/bin/env python3
import math #삼각함수(atan2, asin) 쓰려고

import rclpy
from rclpy.node import Node # ROS2 파이썬 노드를 만들기 위한 기본 라이브러리
from sensor_msgs.msg import Imu #/imu 토픽에서 오는 메시지 타입 (orientation, angular_velocity, linear_acceleration 등이 들어있는 그 구조체)
from std_msgs.msg import Float64 #/gimbal_roll_cmd, /gimbal_pitch_cmd로 내보낼 때 쓰는 단순 숫자 하나짜리 메시지 타입


def quat_to_roll_pitch(x, y, z, w): #쿼터니언(4개 숫자로 3D 회전을 표현하는 방식)을 사람이 이해하기 쉬운 Roll(좌우 기울기), Pitch(앞뒤 기울기) 각도로 변환하는 표준 공식이에요.
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp) #쿼터니언(4개 숫자로 3D 회전을 표현하는 방식)을 사람이 이해하기 쉬운 Roll(좌우 기울기), Pitch(앞뒤 기울기) 각도로 변환하는 표준 공식이에요.

    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    return roll, pitch #Pitch 계산 공식. max(-1.0, min(1.0, ...))로 값을 -1~1 사이로 강제로 잘라주는(clamp) 이유는, 부동소수점 계산 오차로 asin에 1.0000001 같은 값이 들어가면 에러(NaN)가 나기 때문에 안전장치로 넣은 거예요.


def quat_conjugate(q): #Pitch 계산 공식. max(-1.0, min(1.0, ...))로 값을 -1~1 사이로 강제로 잘라주는(clamp) 이유는, 부동소수점 계산 오차로 asin에 1.0000001 같은 값이 들어가면 에러(NaN)가 나기 때문에 안전장치로 넣은 거예요.
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_multiply(q1, q2): #두 회전을 "합성"하는 공식이에요 (일반 숫자 곱셈이 아니라 쿼터니언 전용 곱셈 공식). q1 * q2는 "q2만큼 회전한 다음, q1만큼 더 회전"한 결과예요.
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


class GimbalLevelingController(Node): #ROS2 노드 이름을 gimbal_leveling_controller로 등록
    def __init__(self):
        super().__init__('gimbal_leveling_controller')
        self.declare_parameter('kp', 1.0)
        self.kp = self.get_parameter('kp').value
        self.q0 = None  # 시작 시점의 IMU 자세를 "수평 기준"으로 저장

        self.roll_pub = self.create_publisher(Float64, '/gimbal_roll_cmd', 10)
        self.pitch_pub = self.create_publisher(Float64, '/gimbal_pitch_cmd', 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.get_logger().info(f'gimbal_leveling_controller started, kp={self.kp}')

    def imu_callback(self, msg: Imu):
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)

        if self.q0 is None:
            self.q0 = q
            self.get_logger().info('IMU baseline captured (assumes robot is level right now)')
            return

        # 시작 시점 대비 "상대 회전"만 추출
        q_rel = quat_multiply(quat_conjugate(self.q0), q)
        roll, pitch = quat_to_roll_pitch(*q_rel)

        roll_cmd = max(-0.4363, min(0.4363, -self.kp * roll))
        pitch_cmd = max(-0.4363, min(0.4363, -self.kp * pitch))

        self.get_logger().info(f'roll={roll:.4f} pitch={pitch:.4f}')

        self.roll_pub.publish(Float64(data=roll_cmd))
        self.pitch_pub.publish(Float64(data=pitch_cmd))


def main():
    rclpy.init()
    node = GimbalLevelingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()