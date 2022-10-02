#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from enum import Enum
from std_msgs.msg import UInt8
from turtlebot3_autorace_msgs.msg import MovingParam

class CoreModeDecider():
    def __init__(self):
        # subscribes : invoking object detected
        self.sub_traffic_light = rospy.Subscriber('/detect/traffic_light', UInt8, self.cbInvokedByTrafficLight, queue_size=1)
        self.sub_traffic_sign = rospy.Subscriber('/detect/traffic_sign', UInt8, self.cbInvokedByTrafficSign, queue_size=1)
        self.sub_returned_mode = rospy.Subscriber('/core/returned_mode', UInt8, self.cbReturnedMode, queue_size=1)

        # publishes : decided mode
        self.pub_decided_mode = rospy.Publisher('/core/decided_mode', UInt8, queue_size=1)

        self.InvokedObject = Enum('InvokedObject', 'traffic_light traffic_sign')
        self.TrafficSign = Enum('TrafficSign', 'construction')
        self.CurrentMode = Enum('CurrentMode', 'idle lane_following traffic_light intersection construction parking level_crossing tunnel')

        self.fnInitMode()
    def cbInvokedByTrafficLight(self, traffic_light_type_msg):
            rospy.loginfo("invoke light")
            self.fnDecideMode(self.InvokedObject.traffic_light.value, traffic_light_type_msg)
            rospy.loginfo("Traffic light detected")

    # Invoke if traffic sign is detected
    def cbInvokedByTrafficSign(self, traffic_sign_type_msg):
        rospy.loginfo("invoke sign")       
        self.fnDecideMode(self.InvokedObject.traffic_sign.value, traffic_sign_type_msg)
        rospy.loginfo("Traffic sign detected")
        
    def cbReturnedMode(self, mode):
        rospy.loginfo("Init Mode")
        self.fnInitMode()

    def fnInitMode(self):                                                   # starts only when the program is started initially or any mission is completed
        self.current_mode = self.CurrentMode.lane_following.value
        self.fnPublishMode()

    def fnDecideMode(self, invoked_object, msg_data):                       # starts only when the traffic sign / traffic light is detected & current_mode is lane_following
        rospy.loginfo("DecideMode")
        if self.current_mode == self.CurrentMode.lane_following.value:
            rospy.loginfo("currentmode : lane following")
            if invoked_object == self.InvokedObject.traffic_light.value:    # Traffic Light detected
                self.current_mode = self.CurrentMode.traffic_light.value
            if invoked_object == self.InvokedObject.traffic_sign.value:    # Any Sign detected
                rospy.loginfo("currentmode : any sign detected")
                if msg_data.data == self.TrafficSign.construction.value:
                    rospy.loginfo("currentmode : construction detected")
                    self.current_mode = self.CurrentMode.construction.value
                elif msg_data.data == self.CurrentMode.intersection.value:

                
            else:
                pass

            self.fnPublishMode()
        else:
            pass

    def fnPublishMode(self):
        decided_mode = UInt8()
        decided_mode.data = self.current_mode
        self.pub_decided_mode.publish(decided_mode)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('core_mode_decider')
    node = CoreModeDecider()
    node.main()
