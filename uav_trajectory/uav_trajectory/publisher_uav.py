# Licensed under CECILL-2.1
#
# Author : thibault SCHWEITZER
# email :  thibault.schweitzer@estaca.eu

import rclpy
import numpy as np
from rclpy.node import Node
from uav_trajectory.uavlib import *

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
        
        # generate UWB data for drones :
        uwb_noise = 0.01
        self.uavA.generate_UWB(self.uavB, self.uavC, self.uavD, uwb_noise)
        self.uavB.generate_UWB(self.uavC, self.uavD, self.uavA, uwb_noise)
        self.uavC.generate_UWB(self.uavD, self.uavA, self.uavB, uwb_noise)
        self.uavD.generate_UWB(self.uavA, self.uavB, self.uavC, uwb_noise)
        
        # generate visual detection data for the drones :
        vd_noise = 0.01
        self.uavA.generate_VD(self.uavB, self.uavC, self.uavD, vd_noise)
        self.uavB.generate_VD(self.uavC, self.uavD, self.uavA, vd_noise)
        self.uavC.generate_VD(self.uavD, self.uavA, self.uavB, vd_noise)
        self.uavD.generate_VD(self.uavA, self.uavB, self.uavC, vd_noise)
        
        # generate visual detection data for the drones :
        lc_noise = 0.01
        self.uavA.generate_LC(self.uavB, self.uavC, self.uavD, lc_noise)
        self.uavB.generate_LC(self.uavC, self.uavD, self.uavA, lc_noise)
        self.uavC.generate_LC(self.uavD, self.uavA, self.uavB, lc_noise)
        self.uavD.generate_LC(self.uavA, self.uavB, self.uavC, lc_noise)
        
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        

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
        msg.x = self.uavC.groundtruth_x[self.i]
        msg.y = self.uavC.groundtruth_y[self.i]
        msg.z = self.uavC.groundtruth_z[self.i]
        msg.psi = 0.0
        self.publisher_gt.publish(msg)
        self.get_logger().info('Publishing: groundtruth infos')

    def callback_vio(self):
        msg = Viomeasurement()
        msg.frame_id = self.i
        msg.measure = np.reshape(self.uavA.z_VIO[self.i].T, (1,16))[0]
        self.publisher_vio.publish(msg)
        self.get_logger().info('Publishing: VIO infos')

    def callback_uwb(self):
        msg = Uwbmeasurement()
        msg.frame_id = self.i
        msg.dist_b = self.uavA.z_UWB[self.i]["distance to B"]
        msg.dist_c = self.uavA.z_UWB[self.i]["distance to C"]
        msg.dist_d = self.uavA.z_UWB[self.i]["distance to D"]
        self.publisher_uwb.publish(msg)
        self.get_logger().info('Publishing: UWB infos')

    def callback_vd(self):
        msg = Vdmeasurement()
        msg.frame_id = self.i
        
        if(self.uavA.z_VD[self.i]["B detected"] is not None): # A->B
            msg.b_detected = True
            msg.measure_b = np.reshape(self.uavA.z_VD[self.i]["B detected"], (1,16))[0] 
        else:
            msg.b_detected = False
            msg.measure_b = np.zeros((1,16),dtype=np.float64)[0]
            
        if(self.uavA.z_VD[self.i]["C detected"] is not None): # A->C
            msg.c_detected = True
            msg.measure_c = np.reshape(self.uavA.z_VD[self.i]["C detected"], (1,16))[0] 
        else:
            msg.c_detected = False
            msg.measure_c = np.zeros((1,16),dtype=np.float64)[0]
        
        if(self.uavA.z_VD[self.i]["D detected"] is not None): # A->D
            msg.d_detected = True
            msg.measure_d = np.reshape(self.uavA.z_VD[self.i]["D detected"], (1,16))[0] 
        else:
            msg.d_detected = False
            msg.measure_d = np.zeros((1,16),dtype=np.float64)[0]
        
        self.publisher_vd.publish(msg)
        self.get_logger().info('Publishing: VD infos')

    def callback_lc(self):
        msg = Lcmeasurement()
        msg.frame_id = self.i
        
        #Loop closure detected for drone A Keyframe
        if(self.uavA.z_LC[self.i]['lc detected AKF'] is not None):
            msg.detected_akf    = True
            msg.time_akf        = self.uavA.z_LC[self.i]['lc time AKF']
            msg.link_akf        = self.uavA.z_LC[self.i]['lc detected AKF']
            msg.measure_akf     = np.reshape(self.uavA.z_LC[self.i]['lc measure AKF'], (1,16))[0]
        else:
            msg.detected_akf    = False
            msg.time_akf        = 0
            msg.link_akf        = "none"
            msg.measure_akf     = np.zeros((1,16),dtype=np.float64)[0]
            
        #Loop closure detected for drone B Keyframe
        if(self.uavA.z_LC[self.i]['lc detected BKF'] is not None):
            msg.detected_bkf    = True
            msg.time_bkf        = self.uavA.z_LC[self.i]['lc time BKF']
            msg.link_bkf        = self.uavA.z_LC[self.i]['lc detected BKF']
            msg.measure_bkf     = np.reshape(self.uavA.z_LC[self.i]['lc measure BKF'], (1,16))[0]
        else:
            msg.detected_bkf    = False
            msg.time_bkf        = 0
            msg.link_bkf        = "none"
            msg.measure_bkf     = np.zeros((1,16),dtype=np.float64)[0]
            
        #Loop closure detected for drone C Keyframe
        if(self.uavA.z_LC[self.i]['lc detected CKF'] is not None):
            msg.detected_ckf    = True
            msg.time_ckf        = self.uavA.z_LC[self.i]['lc time CKF']
            msg.link_ckf        = self.uavA.z_LC[self.i]['lc detected CKF']
            msg.measure_ckf     = np.reshape(self.uavA.z_LC[self.i]['lc measure CKF'], (1,16))[0]
        else:
            msg.detected_ckf    = False
            msg.time_ckf        = 0
            msg.link_ckf        = "none"
            msg.measure_ckf     = np.zeros((1,16),dtype=np.float64)[0]
            
        #Loop closure detected for drone D Keyframe
        if(self.uavA.z_LC[self.i]['lc detected DKF'] is not None):
            msg.detected_dkf    = True
            msg.time_dkf        = self.uavA.z_LC[self.i]['lc time DKF']
            msg.link_dkf        = self.uavA.z_LC[self.i]['lc detected DKF']
            msg.measure_dkf     = np.reshape(self.uavA.z_LC[self.i]['lc measure DKF'], (1,16))[0]
        else:
            msg.detected_dkf    = False
            msg.time_dkf        = 0
            msg.link_dkf        = "none"
            msg.measure_dkf     = np.zeros((1,16),dtype=np.float64)[0]
        
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
