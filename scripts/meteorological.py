#!/usr/bin/env python
import os
import sys
import re
import socket
import rospy
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import RelativeHumidity
from sensor_msgs.msg import Temperature
from sense_hat import SenseHat

class Meteorological(object):

    def __init__(self):
        self.robot_host = re.sub("-", "_", socket.gethostname())
        self.fluidPressurePub = rospy.Publisher(self.robot_host + '/meteorological/pressure', FluidPressure, queue_size=10)
        self.relativeHumidityPub = rospy.Publisher(self.robot_host + '/meteorological/humidity', RelativeHumidity, queue_size=10)
        self.temperaturePub = rospy.Publisher(self.robot_host + '/meteorological/temperature', Temperature, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz

        self.enable = False

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        self.enable = True
        self.fluidPressurePub = rospy.Publisher(self.robot_host + '/meteorological/pressure', FluidPressure, queue_size=10)
        self.relativeHumidityPub = rospy.Publisher(self.robot_host + '/meteorological/humidity', RelativeHumidity, queue_size=10)
        self.temperaturePub = rospy.Publisher(self.robot_host + '/meteorological/temperature', Temperature, queue_size=10)

        sense = SenseHat()

        while not rospy.is_shutdown():
            pressure = sense.get_pressure()
            humidity = sense.get_humidity()
            temp = sense.get_temperature()

            message_str = "Pressure: %s Millibars which are %s Pascal; Humidity: %s %%rH; Temperature: %s °C " % (pressure, (pressure*100), humidity, temp)
            rospy.loginfo(message_str)

            f = FluidPressure()

            f.header.stamp = rospy.Time.now()
            f.header.frame_id = "/base_link"
            f.fluid_pressure = pressure * 100 # Millibar to Pascal conversion
            f.variance = 0

            h = RelativeHumidity()

            h.header.stamp = rospy.Time.now()
            h.header.frame_id = "/base_link"
            h.relative_humidity = humidity
            h.variance = 0

            t = Temperature()

            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "/base_link"
            t.temperature = temp
            t.variance = 0
                
            self.fluidPressurePub.publish(f)
            self.relativeHumidityPub.publish(h)
            self.temperaturePub.publish(t)
            self.rate.sleep()

    def stop(self):
        self.enable = False
        self.fluidPressurePub.unregister()
        self.relativeHumidityPub.unregister()
        self.temperaturePub.unregister()

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    node_name = re.sub("-", "_", socket.gethostname()) + "_Meteorological"
    rospy.init_node(node_name, anonymous=False)
    # Go to class functions that do all the heavy lifting.

    meteorological = Meteorological()

    try:
        meteorological.start()
    except rospy.ROSInterruptException:
        meteorological.stop()
        pass
    # Allow ROS to go to all callbacks.
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()