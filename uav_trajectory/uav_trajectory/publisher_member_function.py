# Licensed under CECILL-2.1
#
# Author : thibault SCHWEITZER
# email :  thibault.schweitzer@estaca.eu

import rclpy
import numpy as np
from rclpy.node import Node

from uav_interfaces.msg import Groundtruth
from uav_interfaces.msg import Viomeasurement
from uav_interfaces.msg import Uwbmeasurement
from uav_interfaces.msg import Vdmeasurement
from uav_interfaces.msg import Lcmeasurement

class UavPublisher(Node):

    def __init__(self):
        super().__init__('uav_publisher')
        self.publisher_gt = self.create_publisher(Groundtruth,    'topic1', 10)
        self.publisher_vio = self.create_publisher(Viomeasurement, 'topic2', 10)
        self.publisher_uwb = self.create_publisher(Uwbmeasurement, 'topic3', 10)
        self.publisher_vd = self.create_publisher(Vdmeasurement,  'topic4', 10)
        self.publisher_lc = self.create_publisher(Lcmeasurement,  'topic5', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.npdata = np.array([1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0,16.0])

    def timer_callback(self):
        self.callback_gt()
        self.callback_vio()
        self.callback_uwb()
        self.callback_vd()
        self.callback_lc()
        self.i += 1

    def callback_gt(self):
        msg = Groundtruth()
        msg.frame_id = self.i
        msg.x = 0.891
        msg.y = 0.453
        msg.z = 0.456
        msg.psi = 0.5455545
        self.publisher_gt.publish(msg)
        self.get_logger().info('Publishing: groundtruth infos')

    def callback_vio(self):
        msg = Viomeasurement()
        msg.frame_id = self.i
        msg.measure = self.npdata
        self.publisher_vio.publish(msg)
        self.get_logger().info('Publishing: VIO infos')

    def callback_uwb(self):
        msg = Uwbmeasurement()
        msg.frame_id = self.i
        msg.measure = 58.45425
        self.publisher_uwb.publish(msg)
        self.get_logger().info('Publishing: UWB infos')

    def callback_vd(self):
        msg = Vdmeasurement()
        msg.frame_id = self.i
        msg.measure = self.npdata
        self.publisher_vd.publish(msg)
        self.get_logger().info('Publishing: VD infos')

    def callback_lc(self):
        msg = Lcmeasurement()
        msg.frame_id = self.i
        msg.measure = self.npdata
        self.publisher_lc.publish(msg)
        self.get_logger().info('Publishing: LC infos')

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
