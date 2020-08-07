#!/usr/bin/env python
import os
import sys
import re
import socket
import getpass
import rospy
import message_filters
import threading
from std_msgs.msg import Float64
from robotcar_msgs.msg import Motor
from robotcar_msgs.msg import Steer
from robotcar_msgs.msg import Pan
from robotcar_msgs.msg import Tilt

env=os.path.expanduser(os.path.expandvars('/home/' + getpass.getuser() + '/robotcar/config'))
sys.path.insert(0, env)

from servo_config import ServoConfig

class RobotCarControl(object):

    def __init__(self):
        """Configure subscriber."""
        # Create a subscriber with appropriate topic, custom message and name of
        # callback function.
        self.robot_host = re.sub("-", "_", socket.gethostname())
        
        self.speedSub = None
        self.steerSub = None

        self.motorPub = None
        self.steerPub = None

        self.motorPubRate = rospy.Rate(10) # 10hz
        self.steerPubRate = rospy.Rate(10) # 10hz

        self.speedThread = threading.Thread(target=self.speedContinuation, args=())
        self.steerThread = threading.Thread(target=self.steerContinuation, args=())

        # Initialize message variables.
        self.enable = False
        self.speedData = None
        self.steerData = None
        self.speed = None
        self.steer = None

    def start(self):
        self.enable = True
        self.speedSub = rospy.Subscriber(self.robot_host + '/control/speed', Float64, self.speedCallback)
        self.steerSub = rospy.Subscriber(self.robot_host + '/control/steer', Float64, self.steerCallback)

        self.motorPub = rospy.Publisher(self.robot_host + '/motor/set', Motor, queue_size=10)
        self.steerPub = rospy.Publisher(self.robot_host + '/steer/set', Steer, queue_size=10)

        self.speedThread.start()
        self.steerThread.start()
        
    def stop(self):
        """Turn off subscriber."""
        self.enable = False
        self.motorPub.unregister()
        self.steerPub.unregister()
        self.speedSub.unregister()
        self.steerSub.unregister()
        self.speedThread.join()
        self.steerThread.join()

    def speedCallback(self, data):
        """Handle subscriber data."""
        # Simply print out values in our custom message.
        self.speed = data.data
        
        #msg = "Got speed %s" % self.speed
        #rospy.loginfo(msg)

    def speedContinuation(self):
        self.motorPub = rospy.Publisher(self.robot_host + '/motor/set', Motor, queue_size=10)

        while not rospy.is_shutdown():
            motor = Motor()

            if self.steer != None:
                if self.speed > 0:
                    motor.direction = "forward"
                elif self.speed < 0:
                    motor.direction = "backward"
                elif self.speed == 0:
                    motor.direction = "stopped"
            
                motor.rate = self.speed
            else:
                motor.direction = "stopped"
                motor.rate = 0

            msg = "Publish speed %s and direction %s" % (motor.rate, motor.direction)
            rospy.loginfo(msg)

            self.motorPub.publish(motor)
            self.motorPubRate.sleep()

    def steerCallback(self, data):
        """Handle subscriber data."""
        # Simply print out values in our custom message.
        self.steer = data.data
        
        #msg = "Got steer %s" % self.steer
        #rospy.loginfo(msg)

    def steerContinuation(self):
        conf = ServoConfig()
            
        neutralPWM = conf.getNeutral('steering')
        minPWM = conf.getMin('steering')
        maxPWM = conf.getMax('steering')

        self.steerPub = rospy.Publisher(self.robot_host + '/steer/set', Steer, queue_size=10)

        while not rospy.is_shutdown():
            steer = Steer()

            if self.steer != None:
                if self.steer == 0.0:
                    steer.pwm = neutralPWM
                elif self.steer > 0.0:
                    steer.pwm = 5 * round((neutralPWM + (maxPWM - neutralPWM) * self.steer)/5)
                elif self.steer < 0.0:
                    steer.pwm = 5 * round((neutralPWM + (neutralPWM - minPWM) * self.steer)/5)
            else:
                steer.pwm = neutralPWM

            msg = "Publish steer %s" % steer.pwm
            rospy.loginfo(msg)

            self.steerPub.publish(steer)
            self.steerPubRate.sleep()

if __name__ == '__main__':
    # Initialize the node and name it.
    node_name = re.sub("-", "_", socket.gethostname()) + "_Control"
    rospy.init_node(node_name, anonymous=False)
    
    robotCarControl = RobotCarControl()
    
    # Go to the main loop
    try:
        robotCarControl.start()
        # Wait for messages on topic, go to callback function when new messages arrive.
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    # Stop with Ctrl + C
    except KeyboardInterrupt:
        robotCarControl.stop()

        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")

        for node in nodes:
            os.system("rosnode kill " + node_name)
        
        print("Node stopped")