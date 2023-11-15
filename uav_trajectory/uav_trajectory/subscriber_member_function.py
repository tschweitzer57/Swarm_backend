# Licensed under CECILL-2.1
#
# Author : thibault SCHWEITZER

import rclpy
from rclpy.node import Node

from uav_interfaces.msg import Groundtruth
from uav_interfaces.msg import VIOmeasurement
from uav_interfaces.msg import UWBmeasurement
from uav_interfaces.msg import VDmeasurement
from uav_interfaces.msg import LCmeasurement

class UavSubscriber(Node):

    def __init__(self):
        super().__init__('uav_subscriber')
        self.subscription_gt = self.create_subscription(Groundtruth,'topic1',self.listener_callback_gt,10)
        self.subscription_gt  # prevent unused variable warning
        self.subscription_vio = self.create_subscription(VIOmeasurement,'topic2',self.listener_callback_vio,10)
        self.subscription_vio 
        self.subscription_uwb = self.create_subscription(UWBmeasurement,'topic3',self.listener_callback_uwb,10)
        self.subscription_uwb 
        self.subscription_vd = self.create_subscription(VDmeasurement,'topic4',self.listener_callback_vd,10)
        self.subscription_vd 
        self.subscription_lc = self.create_subscription(LCmeasurement,'topic5',self.listener_callback_lc,10)
        self.subscription_lc

    def listener_callback_gt(self, msg):
        self.get_logger().info('I heard: Groundtruth frame: %d' % msg.frame_id)

    def listener_callback_vio(self, msg):
        self.get_logger().info('I heard: VIO frame: %d' % msg.frame_id)

    def listener_callback_uwb(self, msg):
        self.get_logger().info('I heard: UWB frame: %d' % msg.frame_id)

    def listener_callback_vd(self, msg):
        self.get_logger().info('I heard: VD frame: %d' % msg.frame_id)

    def listener_callback_lc(self, msg):
        self.get_logger().info('I heard: LC data 1: %f' % msg.measure[1])


def main(args=None):
    rclpy.init(args=args)

    uav_subscriber = UavSubscriber()

    rclpy.spin(uav_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    uav_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
