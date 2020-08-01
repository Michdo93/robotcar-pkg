#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Import required Python code.
import os
import sys
import rospy
import getpass
import socket
import re
from robotcar.msg import Steer
from std_msgs import Bool
from std_msgs import Int8
from std_msgs import Float32

env=os.path.expanduser(os.path.expandvars('/home/' + getpass.getuser() + '/robotcar'))
sys.path.insert(0, env)

from servo_config import ServoConfig

env2=os.path.expanduser(os.path.expandvars('/home/' + getpass.getuser() + '/robotcar/driver/actor'))
sys.path.insert(1, env2)

from servo import ServoMotor

class SteerNode(object):
    """Node example class."""

    def __init__(self):
        self.robot_host = re.sub("-", "_", socket.gethostname())

        self.steerSub = rospy.Subscriber(self.robot_host + '/steer/set', Steer, self.steerCallback)

        self.steerPWMSub = rospy.Subscriber(self.robot_host + '/steer/set/pwm', Steer, self.steerCallback)
        self.steerDegressSub = rospy.Subscriber(self.robot_host + '/steer/set/degree', Steer, self.steerDegreeCallback)

        self.steerResetSub = rospy.Subscriber(self.robot_host + '/steer/reset', Bool, self.steerResetCallback)

        self.steerPub = rospy.Publisher(self.robot_host + '/steer/get', Steer, queue_size=10)
        self.steerPWMPub = rospy.Publisher(self.robot_host + '/steer/get/pwm', Steer, queue_size=10)
        self.steerDegreePub = rospy.Publisher(self.robot_host + '/steer/get/degree', Steer, queue_size=10)  

        self.steerIntervallPub = rospy.Publisher(self.robot_host + '/steer/get/intervall', Int8, queue_size=10)
        self.steerIntervallPWMPub = rospy.Publisher(self.robot_host + '/steer/get/intervall/pwm', Int8, queue_size=10)
        self.steerIntervallDegreePub = rospy.Publisher(self.robot_host + '/steer/get/intervall/degree', Float32, queue_size=10)

        self.steerChannelPub = rospy.Publisher(self.robot_host + '/steer/get/channel', Int8, queue_size=10)

        self.steerMinPub = rospy.Publisher(self.robot_host + '/steer/get/min', Steer, queue_size=10)
        self.steerMinPWMPub = rospy.Publisher(self.robot_host + '/steer/get/min/pwm', Steer, queue_size=10)
        self.steerMinDegreePub = rospy.Publisher(self.robot_host + '/steer/get/min/degree', Steer, queue_size=10)

        self.steerMaxPub = rospy.Publisher(self.robot_host + '/steer/get/max', Steer, queue_size=10)
        self.steerMaxPWMPub = rospy.Publisher(self.robot_host + '/steer/get/max/pwm', Steer, queue_size=10)
        self.steerMaxDegreePub = rospy.Publisher(self.robot_host + '/steer/get/max/degree', Steer, queue_size=10)

        self.steerNeutralPub = rospy.Publisher(self.robot_host + '/steer/get/neutral', Steer, queue_size=10)
        self.steerNeutralPWMPub = rospy.Publisher(self.robot_host + '/steer/get/neutral/pwm', Steer, queue_size=10)
        self.steerNeutralDegreePub = rospy.Publisher(self.robot_host + '/steer/get/neutral/degree', Steer, queue_size=10)

        self.steerRangePub = rospy.Publisher(self.robot_host + '/steer/get/range', Int8, queue_size=10)
        self.steerRangePWMPub = rospy.Publisher(self.robot_host + '/steer/get/range/pwm', Int8, queue_size=10)
        self.steerRangeDegreePub = rospy.Publisher(self.robot_host + '/steer/get/range/degree', Float32, queue_size=10)

        self.rate = rospy.Rate(10) # 10hz

        self.conf = ServoConfig()
        self.mg996r = ServoMotor(0, self.conf.getNeutral('steering'), self.conf.getMin('steering'), self.conf.getMax('steering'), 5)

        # Initialize message variables.
        self.enable = False
        self.angle = self.conf.getNeutral('steering')

        self.mg996r.set_current_pwm(self.angle)

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        """Turn on publisher and subscriber."""
        self.enable = True

        self.steerSub = rospy.Subscriber(self.robot_host + '/steer/set', Steer, self.steerCallback)

        self.steerPWMSub = rospy.Subscriber(self.robot_host + '/steer/set/pwm', Steer, self.steerCallback)
        self.steerDegressSub = rospy.Subscriber(self.robot_host + '/steer/set/degree', Steer, self.steerDegreeCallback)

        self.steerResetSub = rospy.Subscriber(self.robot_host + '/steer/reset', Bool, self.steerResetCallback)

        self.steerPub = rospy.Publisher(self.robot_host + '/steer/get', Steer, queue_size=10)
        self.steerPWMPub = rospy.Publisher(self.robot_host + '/steer/get/pwm', Steer, queue_size=10)
        self.steerDegreePub = rospy.Publisher(self.robot_host + '/steer/get/degree', Steer, queue_size=10)  

        self.steerIntervallPub = rospy.Publisher(self.robot_host + '/steer/get/intervall', Int8, queue_size=10)
        self.steerIntervallPWMPub = rospy.Publisher(self.robot_host + '/steer/get/intervall/pwm', Int8, queue_size=10)
        self.steerIntervallDegreePub = rospy.Publisher(self.robot_host + '/steer/get/intervall/degree', Float32, queue_size=10)

        self.steerChannelPub = rospy.Publisher(self.robot_host + '/steer/get/channel', Int8, queue_size=10)

        self.steerMinPub = rospy.Publisher(self.robot_host + '/steer/get/min', Steer, queue_size=10)
        self.steerMinPWMPub = rospy.Publisher(self.robot_host + '/steer/get/min/pwm', Steer, queue_size=10)
        self.steerMinDegreePub = rospy.Publisher(self.robot_host + '/steer/get/min/degree', Steer, queue_size=10)

        self.steerMaxPub = rospy.Publisher(self.robot_host + '/steer/get/max', Steer, queue_size=10)
        self.steerMaxPWMPub = rospy.Publisher(self.robot_host + '/steer/get/max/pwm', Steer, queue_size=10)
        self.steerMaxDegreePub = rospy.Publisher(self.robot_host + '/steer/get/max/degree', Steer, queue_size=10)

        self.steerNeutralPub = rospy.Publisher(self.robot_host + '/steer/get/neutral', Steer, queue_size=10)
        self.steerNeutralPWMPub = rospy.Publisher(self.robot_host + '/steer/get/neutral/pwm', Steer, queue_size=10)
        self.steerNeutralDegreePub = rospy.Publisher(self.robot_host + '/steer/get/neutral/degree', Steer, queue_size=10)

        self.steerRangePub = rospy.Publisher(self.robot_host + '/steer/get/range', Int8, queue_size=10)
        self.steerRangePWMPub = rospy.Publisher(self.robot_host + '/steer/get/range/pwm', Int8, queue_size=10)
        self.steerRangeDegreePub = rospy.Publisher(self.robot_host + '/steer/get/range/degree', Float32, queue_size=10)

        while not rospy.is_shutdown():
            steerMsg = Steer()
            steerDegMsg = Steer()

            steerIntervallMsg = Int8()
            steerIntervallDegMsg = Float32()

            steerChannelMsg = Int8()

            steerMinMsg = Steer()
            steerMinDegMsg = Steer()

            steerMaxMsg = Steer()
            steerMaxDegMsg = Steer()

            steerNeutralMsg = Steer()
            steerNeutralDegMsg = Steer()

            steerRangeMsg = Int8()
            steerRangeDegMsg = Int8()

            steerMsg.angle = self.mg996r.get_current_pwm()
            steerDegMsg.angle = self.mg996r.get_current_degree()

            steerIntervallMsg.data = self.mg996r.get_intervall_pwm()
            steerIntervallDegMsg.data = self.mg996r.get_intervall_degree()

            steerChannelMsg.data = self.mg996r.get_channel()

            steerMinMsg.angle = self.mg996r.get_servo_min()
            steerMinDegMsg.angle = self.mg996r.get_servo_min_degree()

            steerMaxMsg.angle = self.mg996r.get_servo_max()
            steerMaxDegMsg.angle = self.mg996r.get_servo_max_degree()

            steerNeutralMsg.angle = self.mg996r.get_servo_neutral()
            steerNeutralDegMsg.angle = self.mg996r.get_servo_neutral_degree()

            steerRangeMsg.data = self.mg996r.get_servo_range_pwm()
            steerRangeDegMsg.data = self.mg996r.get_servo_range_degree()

            self.steerPub.publish(steerMsg)
            self.steerPWMPub.publish(steerMsg)
            self.steerDegreePub.publish(steerDegMsg)

            self.steerIntervallPub.publish(steerIntervallMsg)
            self.steerIntervallPWMPub.publish(steerIntervallMsg)
            self.steerIntervallDegreePub.publish(steerIntervallDegMsg)

            self.steerChannelPub.publish(steerChannelMsg)

            self.steerMinPub.publish(steerMinMsg)
            self.steerMinPWMPub.publish(steerMinMsg)
            self.steerMinDegreePub.publish(steerMinDegMsg)

            self.steerMaxPub.publish(steerMaxMsg)
            self.steerMaxPWMPub.publish(steerMaxMsg)
            self.steerMaxDegreePub.publish(steerMaxDegMsg)

            self.steerNeutralPub.publish(steerNeutralMsg)
            self.steerNeutralPWMPub.publish(steerNeutralMsg)
            self.steerNeutralDegreePub.publish(steerNeutralDegMsg)

            self.steerRangePub.publish(steerRangeMsg)
            self.steerRangePWMPub.publish(steerRangeMsg)
            self.steerRangeDegreePub.publish(steerRangeDegMsg)

            self.rate.sleep()

    def stop(self):
        """Turn off publisher and subscriber."""
        self.enable = False

        self.steerSub.unregister()

        self.steerPWMSub.unregister()
        self.steerDegressSub.unregister()

        self.steerResetSub.unregister()

        self.steerPub.unregister()
        self.steerPWMPub.unregister()
        self.steerDegreePub.unregister()

        self.steerIntervallPub.unregister()
        self.steerIntervallPWMPub.unregister()
        self.steerIntervallDegreePub.unregister()

        self.steerChannelPub.unregister()

        self.steerMinPub.unregister()
        self.steerMinPWMPub.unregister()
        self.steerMinDegreePub.unregister()

        self.steerMaxPub.unregister()
        self.steerMaxPWMPub.unregister()
        self.steerMaxDegreePub.unregister()

        self.steerNeutralPub.unregister()
        self.steerNeutralPWMPub.unregister()
        self.steerNeutralDegreePub.unregister()

        self.steerRangePub.unregister()
        self.steerRangePWMPub.unregister()
        self.steerRangeDegreePub.unregister()

    def steerCallback(self, data):
        """Handle subscriber data."""
        self.angle = data.data
        self.mg996r.set_current_pwm(self.angle)

        rospy.loginfo(rospy.get_caller_id() + ' Set steering angle to %s', self.angle)

    def steerDegreeCallback(self, data):
        """Handle subscriber data."""
        self.mg996r.set_current_degree(data.data)
        self.angle = self.mg996r.get_current_pwm()

        rospy.loginfo(rospy.get_caller_id() + ' Got steering angle degree %s and set angle to %s' % (data.data, self.angle))

    def steerResetCallback(self, data):
        """Handle subscriber data."""

        if(data.data == True):
            self.angle = self.conf.getNeutral('steering')
            self.mg996r.set_current_pwm(self.angle)

            rospy.loginfo(rospy.get_caller_id() + ' Steering reseted to angle %s', self.angle)

# Main function.
if __name__ == "__main__":
    # Initialize the node and name it.
    node_name = re.sub("-", "_", socket.gethostname()) + "_Steer"
    rospy.init_node(node_name, anonymous=False)

    # Go to class functions that do all the heavy lifting.

    steer = SteerNode()

    try:
        steer.start()
    except rospy.ROSInterruptException:
        steer.stop()
        pass
    # Allow ROS to go to all callbacks.
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
