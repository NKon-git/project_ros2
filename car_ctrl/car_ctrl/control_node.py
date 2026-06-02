#!/usr/bin/env python3
import rclpy #import ros lib for nodes
from rclpy.node import Node
import time
from std_msgs.msg import Float32MultiArray
import smbus2

#creates node
class ControlNode(Node):

    def __init__(self):

        super().__init__("control_node")  #sets the name of the node
        self.get_logger().info('control node starting')

        # parameters
        #control parameters 
        self.declare_parameter('kp', 1)   #p coefficient for speed control
        self.declare_parameter('s_filter', 0.5)   #tells how much weight is on the previous command
        self.P=self.get_parameter('kp').value
        self.speed_filter=self.get_parameter('s_filter').value
        self.previous_speed=0.0
        

        self.declare_parameter('ang_filter',0.5)   #tells how much weight is on the previous angle command
        self.declare_parameter('steering_coef', 0.5)  #sets how many degrees command induces each unit of side error 
        self.declare_parameter('servo_angle_coef', 0.35)   #weighs how much 45/315° angle measures are valued
        self.servo_angle_k=self.get_parameter('servo_angle_coef').value
        self.steering_k=self.get_parameter('steering_coef').value
        self.angle_filter=self.get_parameter('ang_filter').value

        #lidar
        self.lidar0=0
        self.lidar45=0
        self.lidar90=0
        self.lidar270=0
        self.lidar315=0

        self.lidarmin=10 #cm, min range
        self.lidarmax=800 #cm, max lidar range
        
        self.lidar_life=1

        #ultrasound
        self.ultraleft=40
        self.ultramid=200
        self.ultraright=40

        #merged data (mixed lidar and ultrasound)
        self.pre_front_value=200
        self.prev_side_value=70.0

        #servo data
        self.servo0=70
        self.declare_parameter('servo_max', 35) #servo max value from center
        self.servomax=self.get_parameter('servo_max').value
        #limites trouvees: 30 - 110

        self.declare_parameter('esc_channel', 7)
        self.declare_parameter('pwm_frequency', 50)
        esc_channel = self.get_parameter('esc_channel').value
        pwm_frequency = self.get_parameter('pwm_frequency').value

        # hardware init
        #motor
        self.pca = PCA9685Direct(bus=1, address=0x42)
        self.pca.set_frequency(pwm_frequency)
        self.motor = ESC(self.pca, esc_channel)
        self.motor.setpwm(0)
        
        #servo
        self.set_servo_angle(70)
        self.get_logger().info("esc arming")
        self.motor.set_ms(1.0)   # 1ms = minimum
        self.motor.setpwm(1)
        time.sleep(3)
        self.motor.setpwm(0)
        time.sleep(3)
        self.get_logger().info("esc armed")
        #subscribers to sensor data
        lidar_data_sub= self.create_subscription(Float32MultiArray,'/distances/lidar', self.lidar_treat,10)
        ultrasound_data_sub= self.create_subscription(Float32MultiArray,'/distances/ultrasound', self.ultrasound_treat,10)

        #control loop
        self.create_timer(0.05, self.control_callback)

    def set_servo_angle(self, angle):
        angle = max(0, min(180, angle))
        # Map angle to pulse width (1ms-2ms)
        ms = 1.39 + (angle / 180.0)
        duty = int(ms / 20.0 * 65535)
        self.pca.set_duty_cycle(0, duty)  # channel 0

    def lidar_treat(self, msg: Float32MultiArray):
        self.lidar0=max(self.lidarmin,min(msg.data[0],self.lidarmax))
        self.lidar45=max(self.lidarmin,min(msg.data[1],self.lidarmax))
        self.lidar90=max(self.lidarmin,min(msg.data[2],self.lidarmax))
        self.lidar270=max(self.lidarmin,min(msg.data[3],self.lidarmax))
        self.lidar315=max(self.lidarmin,min(msg.data[4],self.lidarmax))
        self.lidar_life=0

    def ultrasound_treat(self, msg: Float32MultiArray):
        self.lidar_life=1
        if msg.data[0]!= -1:
            self.ultraleft=msg.data[0]
        if msg.data[1]!= -1:
            self.ultramid=msg.data[1]
        if msg.data[2]!= -1:
            self.ultraright=msg.data[2]

    def control_callback(self):
        #merging weights
        lidar_weight=max(0.0, 1-self.lidar_life/0.1)

        #merging data
        front_value= (self.lidar0*lidar_weight + self.ultramid)/(1+lidar_weight)

        right_lat = (self.ultraright+self.lidar270*lidar_weight)/(1+lidar_weight)
        left_lat = (self.ultraleft+self.lidar90*lidar_weight)/(1+lidar_weight)

        right_value=right_lat*(1-self.servo_angle_k) + self.lidar45*self.servo_angle_k
        left_value=left_lat*(1-self.servo_angle_k) + self.lidar315*self.servo_angle_k

        #servo control
        side_err = right_value-left_value
        servo_angle = max(70-self.servomax,min(70+side_err*self.steering_k,70+self.servomax))

        #sets servo value
        side_command = servo_angle*(1-self.angle_filter)+self.prev_side_value*self.angle_filter
        self.set_servo_angle(side_command)
        
        #speed control
        speed_command=max(0.0, min(1.0,self.P*(front_value*(1-self.speed_filter)+self.previous_speed*self.speed_filter)/20))
        self.motor.setpwm(speed_command)

        #actualize errors
        self.prev_side_value = side_command
        self.previous_speed = speed_command
        self.lidar_life += 0.05

    def destroy_node(self):
        self.motor.stop()
        self.pca.deinit()
        super().destroy_node()
                       
#esc control class
class ESC:

    #constants in ms 
    NEUTRAL_MS = 1.5 #ms
    PERIOD_MS = 20.0    #signal period for ESC in ms
    MAX_RANGE = 0.3  # ms from neutral
    RESOLUTION = 65535 #duty, 2^16-1
    DEAD_ZONE = 0.01  # ms dead zone around neutral

    def __init__(self, pca_instance, channel):
        self.pca_instance= pca_instance
        self.channel= channel

    def set_ms(self, ms):
        ms = max(1.0, min(2.0, ms))
        duty = int(ms / self.PERIOD_MS * 65535)
        self.pca_instance.set_duty_cycle(self.channel, duty)

    def setpwm(self, taux):
        taux = -taux
        impulsion_ms = self.NEUTRAL_MS + (taux * self.MAX_RANGE)
        if abs(impulsion_ms - self.NEUTRAL_MS) < self.DEAD_ZONE:
            impulsion_ms = self.NEUTRAL_MS
        else:
            impulsion_ms = max(self.NEUTRAL_MS - self.MAX_RANGE, 
                          min(self.NEUTRAL_MS + self.MAX_RANGE, impulsion_ms))
        duty = int(impulsion_ms / self.PERIOD_MS * 65535)
        self.pca_instance.set_duty_cycle(self.channel, duty)

    def stop(self):
        self.setpwm(0)

class PCA9685Direct:
    """Direct smbus2 implementation of PCA9685 control"""
    def __init__(self, bus=1, address=0x42):
        self.bus = smbus2.SMBus(bus)
        self.address = address
        # Reset
        self.bus.write_byte_data(self.address, 0x00, 0x00)
        time.sleep(0.1)
    
    def set_frequency(self, freq):
        # Calculate prescale
        prescale = int(25000000.0 / (4096.0 * freq) - 1)
        # Sleep mode to set prescale
        self.bus.write_byte_data(self.address, 0x00, 0x10)
        self.bus.write_byte_data(self.address, 0xFE, prescale)
        # Wake up
        self.bus.write_byte_data(self.address, 0x00, 0x00)
        time.sleep(0.1)
        self.bus.write_byte_data(self.address, 0x00, 0xa1)

    def set_pwm(self, channel, on, off):
        reg = 0x06 + 4 * channel
        self.bus.write_byte_data(self.address, reg, on & 0xFF)
        self.bus.write_byte_data(self.address, reg + 1, on >> 8)
        self.bus.write_byte_data(self.address, reg + 2, off & 0xFF)
        self.bus.write_byte_data(self.address, reg + 3, off >> 8)

    def set_duty_cycle(self, channel, duty):
        # duty is 0-65535, convert to 0-4095
        off = int(duty * 4095 / 65535)
        self.set_pwm(channel, 0, off)

    def deinit(self):
        self.bus.close()

def main(args=None):
    rclpy.init(args=args) #initiates ros coms
    node = ControlNode()       #creates a node
    try:
        rclpy.spin(node)      #repeats node until stopped manually /!\ IMPORTANT: else the node runs once and stops
    except KeyboardInterrupt:
        pass	
    finally:
        node.destroy_node()
        rclpy.shutdown()      #stops coms

if __name__ == '__main__':
    main()