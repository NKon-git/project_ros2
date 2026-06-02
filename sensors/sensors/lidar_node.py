#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time
from collections import deque
import numpy as np
import threading
from std_msgs.msg import Float32MultiArray

# CRC-8 lookup table from LD14P SDK (polynomial 0x4D)
CRC8_TABLE_LD14P = [
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
    0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
    0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
    0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
    0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
    0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
    0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
    0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
    0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
    0x7f, 0x32, 0xe5, 0xa8
]

def cal_crc8_ld14p(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc = CRC8_TABLE_LD14P[(crc ^ byte) & 0xFF]
    return crc

#classes used in the code

class Packet:
    def __init__(self, raw_data: bytearray):
        self.datapoints = []
        self.radar_speed = None
        self.start_angle = None
        self.end_angle = None
        self.complete = False
        self.datapoints_appended = 0
        self.raw_data = raw_data
        if raw_data[0:1] == LD14P.START_BYTE and raw_data[1:2] == LD14P.VER_LEN:
            self._decode()
        
    def __str__(self) -> str:
        datapoints_string = '\n'.join(d.__repr__() for d in self.datapoints)
        return f"Packet object with data: speed: {self.radar_speed}RPS, \
                angle: {self.start_angle}-{self.end_angle}deg\n{datapoints_string}"

    def _decode(self):
        crc = cal_crc8_ld14p(self.raw_data[:-2])
        if self.raw_data[-2] == crc:
            self.radar_speed = int.from_bytes(self.raw_data[2:4], 'little')
            self.start_angle = int.from_bytes(self.raw_data[4:6], 'little') * 0.01
            self.end_angle = int.from_bytes(self.raw_data[42:44], 'little') * 0.01
            self.timestamp = int.from_bytes(self.raw_data[44:46], 'little')
            # if self.timestamp > 30000:
            #     raise ValueError

            for i in range(6, 42, 3):
                three_bytes = self.raw_data[i:i+3]
                distance = int.from_bytes(three_bytes[0:2], 'little')
                new_datapoint = Datapoint(distance, self.timestamp)
                self.datapoints.append(new_datapoint)
                self.datapoints_appended += 1
            
            if self.start_angle > self.end_angle:
                angle_range = np.linspace(self.start_angle, self.end_angle + 360, num=self.datapoints_appended, endpoint=False)
                angle_range = np.mod(angle_range, 360)  # Normalize angles to [0, 360)
            else:
                angle_range = np.linspace(self.start_angle, self.end_angle, num=self.datapoints_appended, endpoint=False)
            
            for i, d in enumerate(self.datapoints):
                d.angle = round(angle_range[i], 2)
            self.complete = True
        else:
            print(f"CRC NOT CORRECT! crc computed: {crc}, expected crc: {self.raw_data[-2]}")
            LD14P.packets_dropped += 1


class Datapoint:
    def __init__(self, distance, timestamp):
        self.angle = None
        self.distance = distance
        self.timestamp = timestamp

    def __repr__(self) -> str:
        return f"Datapoint object: angle= {self.angle}, distance= {self.distance}, timestamp= {self.timestamp}"

    def to_dict(self):
        return {'timestamp' : self.timestamp,
                'angle' : self.angle,
                'distance' : self.distance}
    

class LD14P:
    START_BYTE = b'\x54'
    VER_LEN = b'\x2c'
    packets_dropped = 0

    def __init__(self, port="/dev/ttyUSB0", baudrate=230400, timeout=0.02):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        self.ser.flush()
        self.ranges = {0: 0, 45: 0, 90: 0, 270: 0, 315: 0}
        self.lock = threading.Lock()


    def update_loop(self):

        # self.t_prev = time.time()
        current_bytes = bytearray()
        last_two_bytes_received = deque([b'\x00', b'\x00'], maxlen=2)
        while True:
            try:
                if self.ser.in_waiting > 0:
                    byte = self.ser.read(1) # Read a byte from the serial port
                    last_two_bytes_received.append(byte)    #Add it to the queue to watch for new packet starting sequence
                    if(last_two_bytes_received[0], last_two_bytes_received[1]) == (LD14P.START_BYTE, LD14P.VER_LEN):    #if starting sequence is detected
                    # print("Start of new packet detected.")
                      
                        if len(current_bytes) > 0:  #if there are already some bytes recorded
                            new_packet = Packet(current_bytes)  #compose them into a packet object
                            if new_packet.complete: #if that's complete
                                # print(f"new packet: {new_packet}, packet dropped: {self.packets_dropped}")
                                self.update_ranges(new_packet)  #process the packet
                            
                        current_bytes.clear()   #clear the array to collect new bytes
                        current_bytes += last_two_bytes_received[0] #add starting byte
                    current_bytes += byte   #add a byte to the current bytearray
            except Exception as e:
                self.get_logger().error(f'Serial error: {e}')
                break

    def update_ranges(self, packet: Packet):
        for dp in packet.datapoints:
            for range_angle in self.ranges.keys():
                if abs(int(dp.angle) - int(range_angle)) < 2:
                    with self.lock:
                        self.ranges[range_angle] = dp.distance/10.0
                    # if range_angle == 0:
                    #     t2 = time.time()
                    #     if(t2 > self.t_prev):
                    #         print(f"Frequency: {1/(t2-self.t_prev)}Hz")
                    #         self.t_prev = t2
                    # print(f"Ranges updated: {self.ranges}")
                    return

#creates node
class LidarNode(Node):
    def __init__(self):
        super().__init__("lidar_node")  #sets the name of the node
        self.get_logger().info("lidar node starting")

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 230400)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        self.lidar = LD14P(port=port, baudrate=baudrate)

        #create thread for the update loop
        loop_thread = threading.Thread(target=self.lidar.update_loop, daemon=True)
        loop_thread.start()

        #publisher and upload values 
        self.lidar_publisher_ = self.create_publisher(Float32MultiArray,'/distances/lidar',10)
        self.create_timer(0.05,self.measure_callback)

        self.get_logger().info("lidar node ready")

    def measure_callback(self):
        msg = Float32MultiArray()
        msg.data = [float(d) for d in self.lidar.ranges]
        self.lidar_publisher_.publish(msg)

    def destroy_node(self):
        self.lidar.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args) #initiates ros coms
    node = LidarNode()       #creates a node
    try:
        rclpy.spin(node)      #repeats node until stopped manually /!\ IMPORTANT: else the node runs once and stops
    except KeyboardInterrupt:
        pass	
    finally:
        node.destroy_node()
        rclpy.shutdown()      #stops coms

if __name__ == '__main__':
    main()