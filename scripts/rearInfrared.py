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

from ir_GP2Y0A02YKOF import IRGP2Y0A02YKOF

class RearInfrared(object):

    def __init__(self):
        self.robot_host = re.sub("-", "_", socket.gethostname())
        self.infraredPub = rospy.Publisher(self.robot_host + '/infrared/rear/distance', Range, queue_size=10)
        self.infraredVelocityPub = rospy.Publisher(self.robot_host + '/infrared/rear/relative_velocity', RelativeVelocity, queue_size=10)
        
        self.rate = rospy.Rate(10) # 10hz

        self.enable = False

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        self.enable = True
        self.infraredPub = rospy.Publisher(self.robot_host + '/infrared/rear/distance', Range, queue_size=10)
        self.infraredVelocityPub = rospy.Publisher(self.robot_host + '/infrared/rear/relative_velocity', RelativeVelocity, queue_size=10)
        
        infrared = IRGP2Y0A02YKOF(1)

        #ranges = [float('NaN'), 1.0, -float('Inf'), 3.0, float('Inf')]
        min_range = 20
        max_range = 150

        while not rospy.is_shutdown():
            
            distance = infrared.distance()
            relative_velocity = infrared.speed()

            message_str = "rearInfrared Distance: %s cm and Speed: %s m/s" % (distance, relative_velocity)
            rospy.loginfo(message_str)
            
            #for distance in ranges:
            r = Range()
            rv = RelativeVelocity()

            r.header.stamp = rospy.Time.now()
            r.header.frame_id = "/base_link"
            r.radiation_type = Range.INFRARED
            r.field_of_view = 0.087266462599716 # 5 degrees
            r.min_range = min_range
            r.max_range = max_range

            r.range = distance
                
            rv.header.stamp = rospy.Time.now()
            rv.header.frame_id = "/base_link"
            rv.radiation_type = Range.INFRARED
            rv.field_of_view = 0.087266462599716 # 5 degrees

            rv.relative_velocity = relative_velocity
                
            self.infraredPub.publish(r)
            self.infraredVelocityPub.publish(rv)   
            self.rate.sleep()

    def stop(self):
        self.enable = False
        self.infraredPub.unregister()
        self.infraredVelocityPub.unregister()

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    node_name = re.sub("-", "_", socket.gethostname()) + "_RearInfrared"
    rospy.init_node(node_name, anonymous=False)
    # Go to class functions that do all the heavy lifting.

    infrared = RearInfrared()

    try:
        infrared.start()
    except rospy.ROSInterruptException:
        infrared.stop()
        pass
    # Allow ROS to go to all callbacks.
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()