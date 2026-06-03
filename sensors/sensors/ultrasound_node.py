#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import lgpio
import time
import threading
from std_msgs.msg import Float32MultiArray

#creates node
class UltrasoundNode(Node):
    def __init__(self):
        super().__init__("ultrasound_node")  #sets the name of the node
        self.get_logger().info("ultrasound node starting")
		
		#pins setup
        self.declare_parameter('trig_pin1', 23)
        self.declare_parameter('trig_pin2', 17)
        self.declare_parameter('trig_pin3', 5)
		
        self.declare_parameter('echo_pin1', 24)
        self.declare_parameter('echo_pin2', 27)
        self.declare_parameter('echo_pin3', 6)
		
        self.sensors = [
        {"TRIG": self.get_parameter('trig_pin1').value,
         "ECHO": self.get_parameter('echo_pin1').value},
        {"TRIG": self.get_parameter('trig_pin2').value,
         "ECHO": self.get_parameter('echo_pin2').value},
        {"TRIG": self.get_parameter('trig_pin3').value,
         "ECHO": self.get_parameter('echo_pin3').value},
        ]
		
        self.h = lgpio.gpiochip_open(4)
        for sensor in self.sensors:
            lgpio.gpio_claim_output(self.h, sensor["TRIG"])
            lgpio.gpio_claim_input(self.h, sensor["ECHO"])
            lgpio.gpio_write(self.h,sensor["TRIG"], 0)

        time.sleep(0.03) #stabilisation initiale
        self.results = [None,None,None]

        #create publisher
        self.ultrasound_pub= self.create_publisher(Float32MultiArray, '/distances/ultrasound', 10)

        self.create_timer(0.05, self.measure_callback)
        
        self.get_logger().info("ultrasound node ready")

    def measure_callback(self): #manage threads for sync measures
        #function to pass to the threads
        def measure_one(index, sensor):
            self.results[index] = measure_distance(self.h,
                sensor["TRIG"],
                sensor["ECHO"]
            )
        
        #create threads
        threads=[]
        for i, sensor in enumerate(self.sensors):
            t = threading.Thread(target=measure_one, args=(i, sensor))
            threads.append(t)
            t.start()   # launch the thread
        
        #sync threads
        for t in threads:
            t.join()
        msg = Float32MultiArray()
        msg.data = [float(d) if d is not None else -1.0 for d in self.results]
        self.ultrasound_pub.publish(msg)

    def destroy_node(self): #properly destroys node
        lgpio.gpiochip_close(self.h)
        super().destroy_node()


#function to calculate distance
def measure_distance(h,TRIG, ECHO):

    lgpio.gpio_write(h,TRIG, 1)
    time.sleep(0.00001)
    lgpio.gpio_write(h,TRIG, 0)

    pulse_start = None
    pulse_end = None

    # Attente du front montant
    timeout = time.time() + 0.05
    while lgpio.gpio_read(h,ECHO) == 0:
        if time.time() > timeout:
            return None
        pulse_start = time.time()

    # Attente du front descendant
    timeout = time.time() + 0.05
    while lgpio.gpio_read(h,ECHO) == 1:
        if time.time() > timeout:
            return None
        pulse_end = time.time()

    if pulse_start is None or pulse_end is None:
        return None

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  #distance en cm
    return round(distance, 2)




def main(args=None):
    rclpy.init(args=args) #initiates ros coms
    node = UltrasoundNode()       #creates a node
    try:
        rclpy.spin(node)      #repeats node until stopped manually /!\ IMPORTANT: else the node runs once and stops
    except KeyboardInterrupt:
        pass	
    finally:
        node.destroy_node()
        rclpy.shutdown()      #stops coms

if __name__ == '__main__':
    main()