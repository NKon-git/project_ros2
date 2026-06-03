#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import busio
import time
from std_msgs.msg import Float32MultiArray

class ControlNode(Node):

    def __init__(self):
        super().__init__("control_node")
        self.get_logger().info('control node starting')

        # control parameters
        self.declare_parameter('kp', 1)
        self.declare_parameter('s_filter', 0.5)
        self.P = self.get_parameter('kp').value
        self.speed_filter = self.get_parameter('s_filter').value
        self.previous_speed = 0.0

        self.declare_parameter('ang_filter', 0.5)
        self.declare_parameter('steering_coef', 0.5)
        self.declare_parameter('servo_angle_coef', 0.35)
        self.servo_angle_k = self.get_parameter('servo_angle_coef').value
        self.steering_k = self.get_parameter('steering_coef').value
        self.angle_filter = self.get_parameter('ang_filter').value

        # lidar
        self.lidar0 = 0
        self.lidar45 = 0
        self.lidar90 = 0
        self.lidar270 = 0
        self.lidar315 = 0
        self.lidarmin = 10
        self.lidarmax = 800
        self.lidar_life = 1

        # ultrasound
        self.ultraleft = 40
        self.ultramid = 200
        self.ultraright = 40

        # merged data
        self.pre_front_value = 200
        self.prev_side_value = 70.0

        # servo data
        self.servo0 = 70
        self.declare_parameter('servo_max', 35)
        self.servomax = self.get_parameter('servo_max').value

        self.declare_parameter('esc_channel', 7)
        self.declare_parameter('servo_channel', 0)
        self.declare_parameter('pwm_frequency', 50)
        esc_channel = self.get_parameter('esc_channel').value
        servo_channel = self.get_parameter('servo_channel').value
        pwm_frequency = self.get_parameter('pwm_frequency').value

        # hardware init
        i2c = busio.I2C(scl=board.SCL, sda=board.SDA)
        self.pca = PCA9685(i2c, address=0x42)
        self.pca.frequency = pwm_frequency
        self.motor = ESC(self.pca, esc_channel)
        self.motor.setpwm(0)

        self.servo_direction = servo.Servo(
            self.pca.channels[servo_channel],
            min_pulse=1000,
            max_pulse=2000
        )
        self.servo_direction.angle = 70

        # esc arming
        self.get_logger().info('arming esc')
        self.motor.setpwm(1)
        time.sleep(3)
        self.motor.setpwm(0)
        time.sleep(3)
        self.get_logger().info('esc armed')

        # subscribers
        self.create_subscription(Float32MultiArray, '/distances/lidar', self.lidar_treat, 10)
        self.create_subscription(Float32MultiArray, '/distances/ultrasound', self.ultrasound_treat, 10)

        # control loop
        self.create_timer(0.05, self.control_callback)

    def lidar_treat(self, msg: Float32MultiArray):
        self.lidar0 = max(self.lidarmin, min(msg.data[0], self.lidarmax))
        self.lidar45 = max(self.lidarmin, min(msg.data[1], self.lidarmax))
        self.lidar90 = max(self.lidarmin, min(msg.data[2], self.lidarmax))
        self.lidar270 = max(self.lidarmin, min(msg.data[3], self.lidarmax))
        self.lidar315 = max(self.lidarmin, min(msg.data[4], self.lidarmax))
        self.lidar_life = 0

    def ultrasound_treat(self, msg: Float32MultiArray):
        if msg.data[0] != -1:
            self.ultraleft = msg.data[0]
        if msg.data[1] != -1:
            self.ultramid = msg.data[1]
        if msg.data[2] != -1:
            self.ultraright = msg.data[2]

    def control_callback(self):
        # merging weights
        lidar_weight = max(0.0, 1 - self.lidar_life / 0.1)

        # merging data
        front_value = (self.lidar0 * lidar_weight + self.ultramid) / (1 + lidar_weight)

        right_lat = (self.ultraright + self.lidar270 * lidar_weight) / (1 + lidar_weight)
        left_lat = (self.ultraleft + self.lidar90 * lidar_weight) / (1 + lidar_weight)

        right_value = right_lat * (1 - self.servo_angle_k) + self.lidar45 * self.servo_angle_k
        left_value = left_lat * (1 - self.servo_angle_k) + self.lidar315 * self.servo_angle_k

        # servo control
        side_err = right_value - left_value
        servo_angle = max(70 - self.servomax, min(70 + side_err * self.steering_k, 70 + self.servomax))
        side_command = servo_angle * (1 - self.angle_filter) + self.prev_side_value * self.angle_filter
        side_command = max(70 - self.servomax, min(70 + self.servomax, side_command))
        self.servo_direction.angle = side_command

        # speed control
        raw_speed = self.P * front_value / 20.0
        speed_command = raw_speed * (1 - self.speed_filter) + self.previous_speed * self.speed_filter
        speed_command = max(0.0, min(1.0, speed_command))
        self.motor.setpwm(speed_command)

        # update state
        self.prev_side_value = side_command
        self.previous_speed = speed_command
        self.lidar_life += 0.05

    def destroy_node(self):
        self.motor.stop()
        self.pca.deinit()
        super().destroy_node()


class ESC:
    NEUTRAL_MS = 1.5
    PERIOD_MS = 20.0
    MAX_RANGE = 0.3
    RESOLUTION = 65535
    DEAD_ZONE = 0.01

    def __init__(self, pca_instance, channel):
        self.pca_instance = pca_instance
        self.channel = channel

    def setpwm(self, taux):
        taux = -taux
        impulsion_ms = self.NEUTRAL_MS + (taux * self.MAX_RANGE)
        if abs(impulsion_ms - self.NEUTRAL_MS) < self.DEAD_ZONE:
            impulsion_ms = self.NEUTRAL_MS
        else:
            impulsion_ms = max(self.NEUTRAL_MS - self.MAX_RANGE,
                               min(self.NEUTRAL_MS + self.MAX_RANGE, impulsion_ms))
        self.pca_instance.channels[self.channel].duty_cycle = int(
            impulsion_ms / self.PERIOD_MS * self.RESOLUTION
        )

    def stop(self):
        self.setpwm(0)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()