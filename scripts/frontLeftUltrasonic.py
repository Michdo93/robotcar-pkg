#!/usr/bin/env python
import os
import sys
import re
import socket
import getpass
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Range
from robotcar_msgs.msg import RelativeVelocity

env=os.path.expanduser(os.path.expandvars('/home/' + getpass.getuser() + '/robotcar/driver/sensor'))
sys.path.insert(0, env)

from ultrasonic_hc_sr04 import UltrasonicHCSR04

class FrontLeftUltrasonic(object):

    def __init__(self):
        self.robot_host = re.sub("-", "_", socket.gethostname())
        self.ultrasonicPub = rospy.Publisher(self.robot_host + '/ultrasonic/front/left/distance', Range, queue_size=10)
        self.ultrasonicVelocityPub = rospy.Publisher(self.robot_host + '/ultrasonic/front/left/relative_velocity', RelativeVelocity, queue_size=10)

        self.rate = rospy.Rate(10) # 10hz

        self.enable = False

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        self.enable = True
        self.ultrasonicPub = rospy.Publisher(self.robot_host + '/ultrasonic/front/left/distance', Range, queue_size=10)
        self.ultrasonicVelocityPub = rospy.Publisher(self.robot_host + '/ultrasonic/front/left/relative_velocity', RelativeVelocity, queue_size=10)
        
        ultrasonic = UltrasonicHCSR04(23, 24)

        #ranges = [float('NaN'), 1.0, -float('Inf'), 3.0, float('Inf')]
        min_range = 3
        max_range = 400

        while not rospy.is_shutdown():
            
            distance = ultrasonic.distance()
            relative_velocity = ultrasonic.speed()

            message_str = "frontLeftUltrasonic Distance: %s cm and Speed: %s m/s" % (distance, relative_velocity)
            rospy.loginfo(message_str)
            
            #for distance in ranges:
            r = Range()
            rv = RelativeVelocity()

            r.header.stamp = rospy.Time.now()
            r.header.frame_id = "/base_link"
            r.radiation_type = Range.ULTRASOUND
            r.field_of_view = 0.26179938779915 # 15 degrees
            r.min_range = min_range
            r.max_range = max_range

            r.range = distance
                
            rv.header.stamp = rospy.Time.now()
            rv.header.frame_id = "/base_link"
            rv.radiation_type = Range.ULTRASOUND
            rv.field_of_view = 0.26179938779915 # 15 degrees
            rv.min_range = min_range
            rv.max_range = max_range

            rv.relative_velocity = relative_velocity
                
            self.ultrasonicPub.publish(r)
            self.ultrasonicVelocityPub.publish(rv)  
            self.rate.sleep()

    def stop(self):
        self.enable = False
        self.ultrasonicPub.unregister()
        self.ultrasonicVelocityPub.unregister()

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    node_name = re.sub("-", "_", socket.gethostname()) + "_FrontLeftUltrasonic"
    rospy.init_node(node_name, anonymous=False)
    # Go to class functions that do all the heavy lifting.

    ultrasonic = FrontLeftUltrasonic()

    try:
        ultrasonic.start()
    except rospy.ROSInterruptException:
        ultrasonic.stop()
        pass
    # Allow ROS to go to all callbacks.
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()