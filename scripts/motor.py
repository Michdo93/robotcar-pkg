#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Import required Python code.
import os
import sys
import rospy
import getpass
import socket
import re
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_msgs.msg import String
from robotcar_msgs.msg import Motor
from robotcar_msgs.msg import DutyCycle
from robotcar_msgs.msg import Rpm
from robotcar_msgs.msg import Velocity

env=os.path.expanduser(os.path.expandvars('/home/' + getpass.getuser() + '/robotcar/driver/actor'))
sys.path.insert(0, env)

from jga25_370 import JGA25_370

class MotorNode(object):
    """Node example class."""

    def __init__(self):
        # ENA, IN1, IN2, (DC_MAX)
        self.motorjga25_370 = JGA25_370(18, 6, 13)

        # Initialize message variables.
        self.enable = False
        self.rate = 0.0
        self.direction = "stopped"

        self.robot_host = re.sub("-", "_", socket.gethostname())

        self.motorSub = rospy.Subscriber(self.robot_host + '/motor/set', Motor, self.motorCallback)
        
        self.motorRateSub = rospy.Subscriber(self.robot_host + '/motor/set/rate', Int8, self.motorCallback)
        self.motorDirectionSub = rospy.Subscriber(self.robot_host + '/motor/set/direction', String, self.motorCallback)

        self.motorMSSub = rospy.Subscriber(self.robot_host + '/motor/set/ms', Velocity, self.motorCallback)
        self.motorKMHSub = rospy.Subscriber(self.robot_host + '/motor/set/kmh', Velocity, self.motorCallback)
        self.motorMPHSub = rospy.Subscriber(self.robot_host + '/motor/set/mph', Velocity, self.motorCallback)

        self.engineStopSub = rospy.Subscriber(self.robot_host + '/motor/engine/stop', Bool, self.motorCallback)

        self.motorPub = rospy.Publisher(self.robot_host + '/motor/get', Motor, queue_size=10)

        self.motorRatePub = rospy.Publisher(self.robot_host + '/motor/get/rate', Int8, queue_size=10)
        self.motorDirectionPub = rospy.Publisher(self.robot_host + '/motor/get/direction', String, queue_size=10)

        self.motorMSPub = rospy.Publisher(self.robot_host + '/motor/get/ms', Velocity, queue_size=10)
        self.motorKMHPub = rospy.Publisher(self.robot_host + '/motor/get/kmh', Velocity, queue_size=10)
        self.motorMPHPub = rospy.Publisher(self.robot_host + '/motor/get/mph', Velocity, queue_size=10)

        self.motorMSMaxPub = rospy.Publisher(self.robot_host + '/motor/get/ms/max', Velocity, queue_size=10)
        self.motorKMHMaxPub = rospy.Publisher(self.robot_host + '/motor/get/kmh/max', Velocity, queue_size=10)
        self.motorMPHMaxPub = rospy.Publisher(self.robot_host + '/motor/get/mph/max', Velocity, queue_size=10)

        self.motorDCMaxPub = rospy.Publisher(self.robot_host + '/motor/get/dc_max', Int8, queue_size=10)
        self.motorDutyCyclePub = rospy.Publisher(self.robot_host + '/motor/get/duty_cycle', DutyCycle, queue_size=10)
        self.motorDutyCycleFrequency = rospy.Publisher(self.robot_host + '/motor/get/duty_cycle/frequency', DutyCycle, queue_size=10)
        self.motorPWMPub = rospy.Publisher(self.robot_host + '/motor/get/pwm', DutyCycle, queue_size=10)

        self.motorRPMPub = rospy.Publisher(self.robot_host + '/motor/get/rpm', Rpm, queue_size=10)
        self.motorRPMCurrentPub = rospy.Publisher(self.robot_host + '/motor/get/rpm/current', Rpm, queue_size=10)
        self.motorRPMMaxPub = rospy.Publisher(self.robot_host + '/motor/get/rpm/max', Rpm, queue_size=10)

        self.ros_rate = rospy.Rate(10) # 10hz

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        """Turn on publisher and subscriber."""
        self.enable = True

        self.motorSub = rospy.Subscriber(self.robot_host + '/motor/set', Motor, self.motorCallback)
        
        self.motorRateSub = rospy.Subscriber(self.robot_host + '/motor/set/rate', Int8, self.motorCallback)
        self.motorDirectionSub = rospy.Subscriber(self.robot_host + '/motor/set/direction', String, self.motorCallback)

        self.motorMSSub = rospy.Subscriber(self.robot_host + '/motor/set/ms', Velocity, self.motorCallback)
        self.motorKMHSub = rospy.Subscriber(self.robot_host + '/motor/set/kmh', Velocity, self.motorCallback)
        self.motorMPHSub = rospy.Subscriber(self.robot_host + '/motor/set/mph', Velocity, self.motorCallback)

        self.engineStopSub = rospy.Subscriber(self.robot_host + '/motor/engine/stop', Bool, self.motorCallback)

        self.motorPub = rospy.Publisher(self.robot_host + '/motor/get', Motor, queue_size=10)

        self.motorRatePub = rospy.Publisher(self.robot_host + '/motor/get/rate', Int8, queue_size=10)
        self.motorDirectionPub = rospy.Publisher(self.robot_host + '/motor/get/direction', String, queue_size=10)

        self.motorMSPub = rospy.Publisher(self.robot_host + '/motor/get/ms', Velocity, queue_size=10)
        self.motorKMHPub = rospy.Publisher(self.robot_host + '/motor/get/kmh', Velocity, queue_size=10)
        self.motorMPHPub = rospy.Publisher(self.robot_host + '/motor/get/mph', Velocity, queue_size=10)

        self.motorMSMaxPub = rospy.Publisher(self.robot_host + '/motor/get/ms/max', Velocity, queue_size=10)
        self.motorKMHMaxPub = rospy.Publisher(self.robot_host + '/motor/get/kmh/max', Velocity, queue_size=10)
        self.motorMPHMaxPub = rospy.Publisher(self.robot_host + '/motor/get/mph/max', Velocity, queue_size=10)

        self.motorDCMaxPub = rospy.Publisher(self.robot_host + '/motor/get/dc_max', Int8, queue_size=10)
        self.motorDutyCyclePub = rospy.Publisher(self.robot_host + '/motor/get/duty_cycle', DutyCycle, queue_size=10)
        self.motorDutyCycleFrequency = rospy.Publisher(self.robot_host + '/motor/get/duty_cycle/frequency', DutyCycle, queue_size=10)
        self.motorPWMPub = rospy.Publisher(self.robot_host + '/motor/get/pwm', DutyCycle, queue_size=10)

        self.motorRPMPub = rospy.Publisher(self.robot_host + '/motor/get/rpm', Rpm, queue_size=10)
        self.motorRPMCurrentPub = rospy.Publisher(self.robot_host + '/motor/get/rpm/current', Rpm, queue_size=10)
        self.motorRPMMaxPub = rospy.Publisher(self.robot_host + '/motor/get/rpm/max', Rpm, queue_size=10)

        while not rospy.is_shutdown():
            motorMsg = Motor()

            motorRateMsg = Int8()
            motorDirectionMsg = String()

            motorMSMsg = Velocity()
            motorKMHMsg = Velocity()
            motorMPHMsg = Velocity()

            motorMSMaxMsg = Velocity()
            motorKMHMaxMsg = Velocity()
            motorMPHMaxMsg = Velocity()

            motorDCMaxMsg = Int8()
            motorDutyCycleFrequencyMsg = DutyCycle()
            motorPWMMsg = DutyCycle()

            motorRPMMsg = Rpm()
            motorRPMMaxMsg = Rpm()

            motorMsg.rate = self.rate
            motorMsg.direction = self.direction

            motorRateMsg.data = self.rate
            motorDirectionMsg.data = self.direction

            motorMSMsg.velocity = self.motorjga25_370.getLinearVelocityMS()
            motorKMHMsg.velocity = self.motorjga25_370.getLinearVelocityKMH()
            motorMPHMsg.velocity = self.motorjga25_370.getLinearVelocityMPH()

            motorMSMaxMsg.velocity = self.motorjga25_370.getMaxLinearVelocityMS()
            motorKMHMaxMsg.velocity = self.motorjga25_370.getMaxLinearVelocityKMH()
            motorMPHMaxMsg.velocity = self.motorjga25_370.getMaxLinearVelocityMPH()

            motorDCMaxMsg.data = self.motorjga25_370.getDCMax()
            motorDutyCycleFrequencyMsg.pwm = self.motorjga25_370.getFrequency()
            motorPWMMsg.pwm = self.motorjga25_370.getMotorPWM()

            motorRPMMsg.rpm = self.motorjga25_370.getRPM()
            motorRPMMaxMsg.rpm = self.motorjga25_370.getRPMMax()

            self.motorPub.publish(motorMsg)

            self.motorRatePub.publish(motorRateMsg)
            self.motorDirectionPub.publish(motorDirectionMsg)

            self.motorMSPub.publish(motorMSMsg)
            self.motorKMHPub.publish(motorKMHMsg)
            self.motorMPHPub.publish(motorMPHMsg)

            self.motorDCMaxPub.publish(motorDCMaxMsg)
            self.motorDutyCyclePub.publish(motorPWMMsg)
            self.motorDutyCycleFrequency.publish(motorDutyCycleFrequencyMsg)
            self.motorPWMPub.publish(motorPWMMsg)

            self.motorRPMPub.publish(motorRPMMsg)
            self.motorRPMCurrentPub.publish(motorRPMMsg)
            self.motorRPMMaxPub.publish(motorRPMMaxMsg)

            self.ros_rate.sleep()

    def stop(self):
        """Turn off publisher and subscriber."""
        self.enable = False
        # self.motorjga25_370.engineStop()

        self.motorSub.unregister()
        
        self.motorRateSub.unregister()
        self.motorDirectionSub.unregister()

        self.motorMSSub.unregister()
        self.motorKMHSub.unregister()
        self.motorMPHSub.unregister()

        self.engineStopSub.unregister()

        self.motorPub.unregister()

        self.motorRatePub.unregister()
        self.motorDirectionPub.unregister()

        self.motorMSPub.unregister()
        self.motorKMHPub.unregister()
        self.motorMPHPub.unregister()

        self.motorMSMaxPub.unregister()
        self.motorKMHMaxPub.unregister()
        self.motorMPHMaxPub.unregister()

        self.motorDCMaxPub.unregister()
        self.motorDutyCyclePub.unregister()
        self.motorDutyCycleFrequency.unregister()
        self.motorPWMPub.unregister()

        self.motorRPMPub.unregister()
        self.motorRPMCurrentPub.unregister()
        self.motorRPMMaxPub.unregister()

    def motorCallback(self, data):
        """Handle subscriber data."""
        self.rate = data.rate
        self.direction = data.direction

        self.motorjga25_370.setMotorRate(self.rate)

        rospy.loginfo(rospy.get_caller_id() + ' Set rate to %s and direction to %s' % (self.rate, self.direction))

    def motorRateCallback(self, data):
        """Handle subscriber data."""
        self.rate = data.data

        self.motorjga25_370.setMotorRate(self.rate)

        rospy.loginfo(rospy.get_caller_id() + ' Set rate to %s', self.rate)

    def motorDirectionCallback(self, data):
        """Handle subscriber data."""
        self.direction = data.data

        self.motorjga25_370.setMotorDirection(self.direction)

        rospy.loginfo(rospy.get_caller_id() + ' Set direction to %s', self.direction)

    def motorMSCallback(self, data):
        ms = data.data

        self.motorjga25_370.setLinearVelocityMS(ms)

        rospy.loginfo(rospy.get_caller_id() + ' Set Motor to %s m/s', ms)

        self.rate = self.motorjga25_370.getRate()
        self.direction = self.motorjga25_370.getMotorDirection()

    def motorKMHCallback(self, data):
        kmh = data.data

        self.motorjga25_370.setLinearVelocityKMH(kmh)

        rospy.loginfo(rospy.get_caller_id() + ' Set Motor to %s kmh', kmh)

        self.rate = self.motorjga25_370.getRate()
        self.direction = self.motorjga25_370.getMotorDirection()

    def motorMPHCallback(self, data):
        mph = data.data

        self.motorjga25_370.setLinearVelocityMPH(mph)

        rospy.loginfo(rospy.get_caller_id() + ' Set Motor to %s mph', mph)

        self.rate = self.motorjga25_370.getRate()
        self.direction = self.motorjga25_370.getMotorDirection()

    def engineStopCallback(self, data):
        """Handle subscriber data."""
        
        if (data.data == True):
            self.rate = 0.0
            self.direction = "stopped"
            self.motorjga25_370.engineStop()

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
