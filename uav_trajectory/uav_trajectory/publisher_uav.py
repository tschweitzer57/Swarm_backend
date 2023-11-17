# Licensed under CECILL-2.1
#
# Author : thibault SCHWEITZER
# email :  thibault.schweitzer@estaca.eu

import rclpy
import numpy as np
from rclpy.node import Node
from uav_trajectory.uavlib import *

from uav_interfaces.msg import Groundtruth
from uav_interfaces.msg import VIOmeasurement
# from uav_interfaces.msg import UWBmeasurement
# from uav_interfaces.msg import VDmeasurement
# from uav_interfaces.msg import LCmeasurement

class UavPublisher(Node):

    def __init__(self):
        super().__init__('uav_publisher')
        self.publisher_gt = self.create_publisher(Groundtruth,    'topic1', 10)
        self.publisher_vio = self.create_publisher(VIOmeasurement, 'topic2', 10)
        # self.publisher_uwb = self.create_publisher(UWBmeasurement, 'topic3', 10)
        # self.publisher_vd = self.create_publisher(VDmeasurement,  'topic4', 10)
        # self.publisher_lc = self.create_publisher(LCmeasurement,  'topic5', 10)

        # Generates trajectory for UAV A
        self.uavA = UAV(initialPose=[0,0,0])
        self.uavA.generate_3DoF_trajectory(diameter = 30)
        # Generates trajectory for UAV B
        self.uavB = UAV(initialPose=[40,0,0])
        self.uavB.generate_3DoF_trajectory(diameter = 30)
        # Generates trajectory for UAV C
        self.uavC = UAV(initialPose=[0,40,0])
        self.uavC.generate_3DoF_trajectory(diameter = 30)
        # Generates trajectory for UAV D
        self.uavD = UAV(initialPose=[40,40,0])
        self.uavD.generate_3DoF_trajectory(diameter = 30)
        
        # generate VIO data for the drones :
        vio_noise = 0.01
        self.uavA.generate_VIO(vio_noise)
        self.uavB.generate_VIO(vio_noise)
        self.uavC.generate_VIO(vio_noise)
        self.uavD.generate_VIO(vio_noise)
        
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        

    def timer_callback(self):
        self.callback_gt()
        self.callback_vio()
        # self.callback_uwb()
        # self.callback_vd()
        # self.callback_lc()
        self.i += 1

    def callback_gt(self):
        msg = Groundtruth()
        msg.frame_id = self.i
        msg.x = self.uavC.groundtruth_x[self.i]
        msg.y = self.uavC.groundtruth_y[self.i]
        msg.z = self.uavC.groundtruth_z[self.i]
        msg.psi = 0.0
        self.publisher_gt.publish(msg)
        self.get_logger().info('Publishing: groundtruth infos')

    def callback_vio(self):
        msg = VIOmeasurement()
        msg.frame_id = self.i
        data = np.reshape(self.uavA.z_VIO[self.i].T, (1,16))[0]
        print(type(data[0]))
        print(type(data[1]))
        print(type(data[2]))
        print(type(data[3]))
        print(type(data[4]))
        print(type(data[5]))
        print(type(data[6]))
        print(type(data[7]))
        msg.measure = np.array([data[0],2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0,16.0])
        self.publisher_vio.publish(msg)
        self.get_logger().info('Publishing: VIO infos')

    # def callback_uwb(self):
        # msg = UWBmeasurement()
        # msg.frame_id = self.i
        # msg.measure = 58.45425
        # self.publisher_uwb.publish(msg)
        # self.get_logger().info('Publishing: UWB infos')

    # def callback_vd(self):
        # msg = VDmeasurement()
        # msg.frame_id = self.i
        # msg.measure = self.npdata
        # self.publisher_vd.publish(msg)
        # self.get_logger().info('Publishing: VD infos')

    # def callback_lc(self):
        # msg = LCmeasurement()
        # msg.frame_id = self.i
        # msg.measure = self.npdata
        # self.publisher_lc.publish(msg)
        # self.get_logger().info('Publishing: LC infos')

def main(args=None):
    rclpy.init(args=args)

    uav_publisher = UavPublisher()

    rclpy.spin(uav_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    uav_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
