#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Import required Python code.
import os
import sys
import rospy
import getpass
import socket
import re
from robotcar.msg import Motor
from std_msgs import Bool
from std_msgs import Int8
from std_msgs import Float32

env=os.path.expanduser(os.path.expandvars('/home/' + getpass.getuser() + '/robotcar'))
sys.path.insert(0, env)

from servo_config import ServoConfig

env2=os.path.expanduser(os.path.expandvars('/home/' + getpass.getuser() + '/robotcar/driver/actor'))
sys.path.insert(1, env2)

from jga25-370 import JGA25_370

class SteerNode(object):
    """Node example class."""

    def __init__(self):
        self.robot_host = re.sub("-", "_", socket.gethostname())

        self.motorSub = rospy.Subscriber(self.robot_host + '/motor/set', Motor, self.motorCallback)
        
        self.motorRateSub = rospy.Subscriber(self.robot_host + '/motor/set/rate', Motor, self.motorCallback)
        self.motorDirectionSub = rospy.Subscriber(self.robot_host + '/motor/set/direction', Motor, self.motorCallback)

        self.motorMSSub = rospy.Subscriber(self.robot_host + '/motor/set/ms', Motor, self.motorCallback)
        self.motorKMHSub = rospy.Subscriber(self.robot_host + '/motor/set/kmh', Motor, self.motorCallback)
        self.motorMPHSub = rospy.Subscriber(self.robot_host + '/motor/set/mph', Motor, self.motorCallback)

        self.engineStopSub = rospy.Subscriber(self.robot_host + '/motor/engine/stop', Motor, self.motorCallback)

        self.motorPub = rospy.Publisher(self.robot_host + '/motor/get', Int8, queue_size=10)

        self.motorRatePub = rospy.Publisher(self.robot_host + '/motor/get/rate', Int8, queue_size=10)
        self.motorDirectionPub = rospy.Publisher(self.robot_host + '/motor/get/direction', Int8, queue_size=10)

        self.motorMSPub = rospy.Publisher(self.robot_host + '/motor/get/ms', Int8, queue_size=10)
        self.motorKMHPub = rospy.Publisher(self.robot_host + '/motor/get/kmh', Int8, queue_size=10)
        self.motorMPHPub = rospy.Publisher(self.robot_host + '/motor/get/mph', Int8, queue_size=10)

        self.motorDCMaxPub = rospy.Publisher(self.robot_host + '/motor/get/dc_max', Int8, queue_size=10)
        self.motorDutyCyclePub = rospy.Publisher(self.robot_host + '/motor/get/duty_cycle', Int8, queue_size=10)
        self.motorPWMPub = rospy.Publisher(self.robot_host + '/motor/get/pwm', Int8, queue_size=10)

        self.rate = rospy.Rate(10) # 10hz
        
        # ENA, IN1, IN2, (DC_MAX)
        self.jga25_370 = JGA25_370(18, 6, 13)

        # Initialize message variables.
        self.enable = False
        self.rate = 0.0
        self.direction = "stopped"

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        """Turn on publisher and subscriber."""
        self.enable = True

        self.sub = rospy.Subscriber(self.robot_host + '/steer/set', Motor, self.motorCallback)

        while not rospy.is_shutdown():
            motorMsg = Motor()

            self.rate.sleep()

    def stop(self):
        """Turn off publisher and subscriber."""
        self.enable = False

        self.sub.unregister()

    def motorCallback(self, data):
        """Handle subscriber data."""
        self.angle = data.data
        self.mg996r.set_current_pwm(self.angle)

        rospy.loginfo(rospy.get_caller_id() + ' Set steering angle to %s', self.angle)

# Main function.
if __name__ == "__main__":
    # Initialize the node and name it.
    node_name = re.sub("-", "_", socket.gethostname()) + "_Motor"
    rospy.init_node(node_name, anonymous=False)

    # Go to class functions that do all the heavy lifting.

    motor = MotorNode()

    try:
        motor.start()
    except rospy.ROSInterruptException:
        motor.stop()
        pass
    # Allow ROS to go to all callbacks.
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
