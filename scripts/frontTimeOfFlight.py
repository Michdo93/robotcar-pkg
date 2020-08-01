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

from tof_vl53l1x import ToFVL53L1X

class FrontTimeOfFlight(object):
    
    def __init__(self):
        self.robot_host = re.sub("-", "_", socket.gethostname())
        self.tofPub = rospy.Publisher(self.robot_host + '/time_of_flight/front/distance', Range, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz

        self.enable = False

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        self.enable = True
        self.tofPub = rospy.Publisher(self.robot_host + '/time_of_flight/front/distance', Range, queue_size=10)

        tof = ToFVL53L1X(0x28, 25)
        #tof = ToFVL53L1X(0x2a, 12)

        tof.start_sensor(25)
        #tof.start_sensor(12)

        tof.set_range("short")

        #ranges = [float('NaN'), 1.0, -float('Inf'), 3.0, float('Inf')]

        min_range = 4.0
        max_range = 0.0

        if tof.get_range() == "short":
            max_range = 130.0
        elif tof.get_range() == "medium":
            max_range = 300.0
        elif tof.get_range() == "long":
            max_range = 400.0

        while not rospy.is_shutdown():

            distance = tof.get_distance()
            distance = float(distance)

            message_str = "frontTimeOfFlight Distance: %s cm" % distance
            rospy.loginfo(message_str)
            
            #for distance in ranges:
            r = Range()

            r.header.stamp = rospy.Time.now()
            r.header.frame_id = "/base_link"
            r.radiation_type = Range.INFRARED
            r.field_of_view = 0.471239 # 27 degrees
            r.min_range = min_range
            r.max_range = max_range

            r.range = distance
                
            self.tofPub.publish(r)
            self.rate.sleep()

    def stop(self):
        self.enable = False
        self.tofPub.unregister()

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    node_name = re.sub("-", "_", socket.gethostname()) + "_FrontTimeOfFlight"
    rospy.init_node(node_name, anonymous=False)
    # Go to class functions that do all the heavy lifting.

    tof = FrontTimeOfFlight()

    try:
        tof.start()
    except rospy.ROSInterruptException:
        tof.stop()
        pass
    # Allow ROS to go to all callbacks.
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()