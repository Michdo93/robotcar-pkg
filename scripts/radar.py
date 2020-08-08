#!/usr/bin/env python
import os
import sys
import re
import socket
import getpass
import rospy
from sensor_msgs.msg import Range
from robotcar_msgs.msg import RelativeVelocity

env=os.path.expanduser(os.path.expandvars('/home/' + getpass.getuser() + '/robotcar/driver/sensor'))
sys.path.insert(0, env)

#from ir_GP2Y0A02YKOF import IRGP2Y0A02YKOF

class FrontInfrared():
    
    def __init__(self):
        self.robot_host = re.sub("-", "_", socket.gethostname())
        self.radarPub = rospy.Publisher(self.robot_host + '/radar', RelativeVelocity, queue_size=10)
        self.radarVelocityPub = rospy.Publisher(self.robot_host + '/radar/relative_velocity', RelativeVelocity, queue_size=10)

        self.rate = rospy.Rate(10) # 10hz

        self.enable = False

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        self.enable = True
        self.radarPub = rospy.Publisher(self.robot_host + '/radar', RelativeVelocity, queue_size=10)
        self.radarVelocityPub = rospy.Publisher(self.robot_host + '/radar/relative_velocity', RelativeVelocity, queue_size=10)
        
        #radar = #

        while not rospy.is_shutdown():
            
            # relative_velocity = radar.measureSpeed()
            relative_velocity = 0

            message_str = "radar Speed: %s m/s" % relative_velocity
            rospy.loginfo(message_str)
            
            #for distance in ranges:
            rv = RelativeVelocity()

            rv.header.stamp = rospy.Time.now()
            rv.header.frame_id = "/base_link"
            rv.radiation_type = Range.ULTRASOUND
            rv.field_of_view = 0.087266462599716 # 5 degrees

            rv.relative_velocity = relative_velocity
                
            self.radarPub.publish(rv)
            self.radarVelocityPub.publish(rv)
            self.rate.sleep()

    def stop(self):
        self.enable = False
        self.radarPub.unregister()
        self.radarVelocityPub.unregister()

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    node_name = re.sub("-", "_", socket.gethostname()) + "_Radar"
    rospy.init_node(node_name, anonymous=False)
    # Go to class functions that do all the heavy lifting.

    radar = Radar()

    try:
        radar.start()
    except rospy.ROSInterruptException:
        radar.stop()
        pass
    # Allow ROS to go to all callbacks.
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()