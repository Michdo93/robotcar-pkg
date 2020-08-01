#!/usr/bin/env python
import os
import sys
import re
import socket
import getpass
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Range

env=os.path.expanduser(os.path.expandvars('/home/' + getpass.getuser() + '/robotcar/driver/sensor'))
sys.path.insert(0, env)

from ultrasonic_hc_sr04 import UltrasonicHCSR04

class FrontRightUltrasonic(object):

    def __init__(self):
        self.robot_host = re.sub("-", "_", socket.gethostname())
        self.ultrasonicPub = rospy.Publisher(self.robot_host + '/ultrasonic/front/right/distance', Range, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz

        self.enable = False

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        self.enable = True
        self.ultrasonicPub = rospy.Publisher(self.robot_host + '/ultrasonic/front/right/distance', Range, queue_size=10)
        
        ultrasonic = UltrasonicHCSR04(22, 5)

        #ranges = [float('NaN'), 1.0, -float('Inf'), 3.0, float('Inf')]
        min_range = 3
        max_range = 400

        while not rospy.is_shutdown():
            
            distance = ultrasonic.distance()

            message_str = "frontRightUltrasonic Distance: %s cm" % distance
            rospy.loginfo(message_str)
            
            #for distance in ranges:
            r = Range()

            r.header.stamp = rospy.Time.now()
            r.header.frame_id = "/base_link"
            r.radiation_type = Range.ULTRASOUND
            r.field_of_view = 0.26179938779915 # 15 degrees
            r.min_range = min_range
            r.max_range = max_range

            r.range = distance
                
            self.ultrasonicPub.publish(r)    
            self.rate.sleep()

    def stop(self):
        self.enable = False
        self.ultrasonicPub.unregister()

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    node_name = re.sub("-", "_", socket.gethostname()) + "_FrontRightUltrasonic"
    rospy.init_node(node_name, anonymous=False)
    # Go to class functions that do all the heavy lifting.

    ultrasonic = FrontRightUltrasonic()

    try:
        ultrasonic.start()
    except rospy.ROSInterruptException:
        ultrasonic.stop()
        pass
    # Allow ROS to go to all callbacks.
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()