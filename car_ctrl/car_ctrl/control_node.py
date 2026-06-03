#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import lgpio
from std_msgs.msg import Float32MultiArray
#updated
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
        self.declare_parameter('servo_max', 35)
        self.servomax = self.get_parameter('servo_max').value

        self.declare_parameter('esc_gpio', 18)
        self.declare_parameter('servo_gpio', 12)
        self.esc_gpio = self.get_parameter('esc_gpio').value
        self.servo_gpio = self.get_parameter('servo_gpio').value

        # hardware init
        self.h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.h, self.esc_gpio)
        lgpio.gpio_claim_output(self.h, self.servo_gpio)

        # set initial PWM — 50Hz, neutral pulse
        self.set_us(self.esc_gpio, 1500)
        self.set_us(self.servo_gpio, 1500)

        # esc arming
        self.get_logger().info('arming esc')
        self.set_us(self.esc_gpio, 2000)
        time.sleep(3)
        self.set_us(self.esc_gpio, 1500)
        time.sleep(3)
        self.get_logger().info('esc armed')

        # subscribers
        self.create_subscription(Float32MultiArray, '/distances/lidar', self.lidar_treat, 10)
        self.create_subscription(Float32MultiArray, '/distances/ultrasound', self.ultrasound_treat, 10)

        # control loop
        self.create_timer(0.05, self.control_callback)
        self.get_logger().info('control node ready')

    def set_us(self, gpio, pulse_us):
        """Set PWM pulse width in microseconds at 50Hz"""
        # 50Hz = 20000us period
        # duty cycle in millionths (0-1000000)
        duty = int(pulse_us / 20000.0 * 1000000)
        lgpio.tx_pwm(self.h, gpio, 50, duty / 10000.0)

    def ms_to_us(self, ms):
        return int(ms * 1000)

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

        # map angle to pulse width (70° center = 1500us)
        servo_us = int(1500 + (side_command - 70) * (500 / self.servomax))
        servo_us = max(1000, min(2000, servo_us))
        self.set_us(self.servo_gpio, servo_us)

        # speed control
        raw_speed = self.P * front_value / 20.0
        speed_command = raw_speed * (1 - self.speed_filter) + self.previous_speed * self.speed_filter
        speed_command = max(0.0, min(1.0, speed_command))

        # map speed to pulse width (0=1500us neutral, 1=2000us full forward)
        esc_us = int(1500 + speed_command * 500)
        self.set_us(self.esc_gpio, esc_us)

        # update state
        self.prev_side_value = side_command
        self.previous_speed = speed_command
        self.lidar_life += 0.05

    def destroy_node(self):
        self.set_us(self.esc_gpio, 1500)
        self.set_us(self.servo_gpio, 1500)
        lgpio.gpiochip_close(self.h)
        super().destroy_node()


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