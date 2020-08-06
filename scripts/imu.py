#!/usr/bin/env python
import os
import sys
import re
import socket
import rospy
from sense_hat import SenseHat
from std_header_msgs.msg import Float64
from sensor_msgs.msg import Imu
from robotcar_msgs.msg import Accelerometer
from robotcar_msgs.msg import Gyroscope
from robotcar_msgs.msg import Magnetometer
from robotcar_msgs.msg import Orientation
from robotcar_msgs.msg import Pitch
from robotcar_msgs.msg import Roll
from robotcar_msgs.msg import Yaw

class IMU(object):

    def __init__(self):
        self.robot_host = re.sub("-", "_", socket.gethostname())

        self.imuPub = rospy.Publisher(self.robot_host + '/imu', Imu, queue_size=10)
        self.imuRawPub = rospy.Publisher(self.robot_host + '/imu/raw', Imu, queue_size=10)
        
        self.accelerometerPub = rospy.Publisher(self.robot_host + '/imu/accelerometer', Accelerometer, queue_size=10)
        self.accelerometerPitchPub = rospy.Publisher(self.robot_host + '/imu/accelerometer/pitch', Pitch, queue_size=10)
        self.accelerometerRollPub = rospy.Publisher(self.robot_host + '/imu/accelerometer/roll', Roll, queue_size=10)
        self.accelerometerYawPub = rospy.Publisher(self.robot_host + '/imu/accelerometer/yaw', Yaw, queue_size=10)
        self.accelerometerRawPub = rospy.Publisher(self.robot_host + '/imu/accelerometer/raw', Accelerometer, queue_size=10)
        self.accelerometerRawXPub = rospy.Publisher(self.robot_host + '/imu/accelerometer/raw/x', Float64, queue_size=10)
        self.accelerometerRawYPub = rospy.Publisher(self.robot_host + '/imu/accelerometer/raw/y', Float64, queue_size=10)
        self.accelerometerRawZPub = rospy.Publisher(self.robot_host + '/imu/accelerometer/raw/z', Float64, queue_size=10)

        self.gyroscopePub = rospy.Publisher(self.robot_host + '/imu/gyroscope', Gyroscope, queue_size=10)
        self.gyroscopePitchPub = rospy.Publisher(self.robot_host + '/imu/gyroscope/pitch', Pitch, queue_size=10)
        self.gyroscopeRollPub = rospy.Publisher(self.robot_host + '/imu/gyroscope/roll', Roll, queue_size=10)
        self.gyroscopeYawPub = rospy.Publisher(self.robot_host + '/imu/gyroscope/yaw', Yaw, queue_size=10)
        self.gyroscopeRawPub = rospy.Publisher(self.robot_host + '/imu/gyroscope/raw', Gyroscope, queue_size=10)
        self.gyroscopeRawXPub = rospy.Publisher(self.robot_host + '/imu/gyroscope/raw/x', Float64, queue_size=10)
        self.gyroscopeRawYPub = rospy.Publisher(self.robot_host + '/imu/gyroscope/raw/y', Float64, queue_size=10)
        self.gyroscopeRawZPub = rospy.Publisher(self.robot_host + '/imu/gyroscope/raw/z', Float64, queue_size=10)

        self.magnetometerPub = rospy.Publisher(self.robot_host + '/imu/magnetometer', Magnetometer, queue_size=10)
        self.magnetometerRawPub = rospy.Publisher(self.robot_host + '/imu/magnetometer/raw', Orientation, queue_size=10)

        self.orientationPub = rospy.Publisher(self.robot_host + '/imu/orientation', Orientation, queue_size=10)
        self.orientationDegreePub = rospy.Publisher(self.robot_host + '/imu/orientation/degrees', Orientation, queue_size=10)
        self.orientationRadiansPub = rospy.Publisher(self.robot_host + '/imu/orientation/radians', Orientation, queue_size=10)
        self.orientationNorthPub = rospy.Publisher(self.robot_host + '/imu/orientation/north', Magnetometer, queue_size=10)

        self.rate = rospy.Rate(10) # 10hz

        self.enable = False

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        self.enable = True

        self.imuPub = rospy.Publisher(self.robot_host + '/imu', Imu, queue_size=10)
        self.imuRawPub = rospy.Publisher(self.robot_host + '/imu/raw', Imu, queue_size=10)
        
        self.accelerometerPub = rospy.Publisher(self.robot_host + '/imu/accelerometer', Accelerometer, queue_size=10)
        self.accelerometerPitchPub = rospy.Publisher(self.robot_host + '/imu/accelerometer/pitch', Pitch, queue_size=10)
        self.accelerometerRollPub = rospy.Publisher(self.robot_host + '/imu/accelerometer/roll', Roll, queue_size=10)
        self.accelerometerYawPub = rospy.Publisher(self.robot_host + '/imu/accelerometer/yaw', Yaw, queue_size=10)
        self.accelerometerRawPub = rospy.Publisher(self.robot_host + '/imu/accelerometer/raw', Accelerometer, queue_size=10)
        self.accelerometerRawXPub = rospy.Publisher(self.robot_host + '/imu/accelerometer/raw/x', Float64, queue_size=10)
        self.accelerometerRawYPub = rospy.Publisher(self.robot_host + '/imu/accelerometer/raw/y', Float64, queue_size=10)
        self.accelerometerRawZPub = rospy.Publisher(self.robot_host + '/imu/accelerometer/raw/z', Float64, queue_size=10)

        self.gyroscopePub = rospy.Publisher(self.robot_host + '/imu/gyroscope', Gyroscope, queue_size=10)
        self.gyroscopePitchPub = rospy.Publisher(self.robot_host + '/imu/gyroscope/pitch', Pitch, queue_size=10)
        self.gyroscopeRollPub = rospy.Publisher(self.robot_host + '/imu/gyroscope/roll', Roll, queue_size=10)
        self.gyroscopeYawPub = rospy.Publisher(self.robot_host + '/imu/gyroscope/yaw', Yaw, queue_size=10)
        self.gyroscopeRawPub = rospy.Publisher(self.robot_host + '/imu/gyroscope/raw', Gyroscope, queue_size=10)
        self.gyroscopeRawXPub = rospy.Publisher(self.robot_host + '/imu/gyroscope/raw/x', Float64, queue_size=10)
        self.gyroscopeRawYPub = rospy.Publisher(self.robot_host + '/imu/gyroscope/raw/y', Float64, queue_size=10)
        self.gyroscopeRawZPub = rospy.Publisher(self.robot_host + '/imu/gyroscope/raw/z', Float64, queue_size=10)

        self.magnetometerPub = rospy.Publisher(self.robot_host + '/imu/magnetometer', Magnetometer, queue_size=10)
        self.magnetometerRawPub = rospy.Publisher(self.robot_host + '/imu/magnetometer/raw', Orientation, queue_size=10)

        self.orientationPub = rospy.Publisher(self.robot_host + '/imu/orientation', Orientation, queue_size=10)
        self.orientationDegreePub = rospy.Publisher(self.robot_host + '/imu/orientation/degrees', Orientation, queue_size=10)
        self.orientationRadiansPub = rospy.Publisher(self.robot_host + '/imu/orientation/radians', Orientation, queue_size=10)
        self.orientationNorthPub = rospy.Publisher(self.robot_host + '/imu/orientation/north', Magnetometer, queue_size=10)

        sense = SenseHat()

        while not rospy.is_shutdown():
            accel_only = sense.get_accelerometer()
            accel_raw = sense.get_accelerometer_raw()

            gyro_only = sense.get_gyroscope()
            gyro_raw = sense.get_gyroscope_raw()

            north = sense.get_compass()
            compass = sense.get_compass_raw()

            orientation = sense.get_orientation()
            orientation_deg = sense.get_orientation_degrees()
            orientation_rad = sense.get_orientation_radians()

            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "/base_link"
            imu_msg.linear_acceleration.x = accel_only['pitch']
            imu_msg.linear_acceleration.y = accel_only['roll']
            imu_msg.linear_acceleration.z = accel_only['yaw']
            imu_msg.angular_velocity.x = gyro_only['pitch']
            imu_msg.angular_velocity.y = gyro_only['roll']
            imu_msg.angular_velocity.z = gyro_only['yaw']
            imu_msg.orientation.x = orientation['pitch']
            imu_msg.orientation.y = orientation['roll']
            imu_msg.orientation.z = orientation['yaw']
            imu_msg.orientation.w = 0
            imu_msg.orientation_covariance = [99999.9 , 0.0 , 0.0, 0.0, 99999.9, 0.0, 0.0, 0.0, 99999.9]
            imu_msg.angular_velocity_covariance = [0.0, 0.0 , 0.0, 0.0 , 0.0, 0.0, 0.0 , 0.0 , 0.0]
            imu_msg.linear_acceleration_covariance = [0.0 , 0.0 , 0.0, 0.0 , 0.0, 0.0, 0.0 , 0.0 , 0.0]

            imu_raw_msg = Imu()
            imu_raw_msg.header.stamp = rospy.Time.now()
            imu_raw_msg.header.frame_id = "/base_link"
            imu_raw_msg.linear_acceleration.x = accel_raw['x']
            imu_raw_msg.linear_acceleration.y = accel_raw['y']
            imu_raw_msg.linear_acceleration.z = accel_raw['z']
            imu_raw_msg.angular_velocity.x = gyro_raw['x']
            imu_raw_msg.angular_velocity.y = gyro_raw['y']
            imu_raw_msg.angular_velocity.z = gyro_raw['z']
            imu_raw_msg.orientation.x = compass['x']
            imu_raw_msg.orientation.y = compass['y']
            imu_raw_msg.orientation.z = compass['z']
            imu_raw_msg.orientation.w = north
            imu_raw_msg.orientation_covariance = [99999.9 , 0.0 , 0.0, 0.0, 99999.9, 0.0, 0.0, 0.0, 99999.9]
            imu_raw_msg.angular_velocity_covariance = [0.0, 0.0 , 0.0, 0.0 , 0.0, 0.0, 0.0 , 0.0 , 0.0]
            imu_raw_msg.linear_acceleration_covariance = [0.0 , 0.0 , 0.0, 0.0 , 0.0, 0.0, 0.0 , 0.0 , 0.0]

            accel_msg = Accelerometer()
            accel_msg.header.stamp = rospy.Time.now()
            accel_msg.header.frame_id = "/base_link"
            accel_msg.x = accel_only['pitch']
            accel_msg.y = accel_only['roll']
            accel_msg.z = accel_only['yaw']


            accel_pitch_msg = Pitch()
            accel_pitch_msg.header.stamp = rospy.Time.now()
            accel_pitch_msg.header.frame_id = "/base_link"
            accel_pitch_msg.data = accel_only['pitch']

            accel_roll_msg = Roll()
            accel_roll_msg.header.stamp = rospy.Time.now()
            accel_roll_msg.header.frame_id = "/base_link"
            accel_roll_msg.data = accel_only['roll']

            accel_yaw_msg = Yaw()
            accel_yaw_msg.header.stamp = rospy.Time.now()
            accel_yaw_msg.header.frame_id = "/base_link"
            accel_yaw_msg.data = accel_only['yaw']

            accel_raw_msg = Accelerometer()
            accel_raw_msg.header.stamp = rospy.Time.now()
            accel_raw_msg.header.frame_id = "/base_link"
            accel_raw_msg.x = accel_raw['x']
            accel_raw_msg.y = accel_raw['y']
            accel_raw_msg.z = accel_raw['z']

            accel_raw_x_msg = Float64()
            accel_raw_x_msg.header.stamp = rospy.Time.now()
            accel_raw_x_msg.header.frame_id = "/base_link"
            accel_raw_x_msg.data = accel_raw['x']

            accel_raw_y_msg = Float64()
            accel_raw_y_msg.header.stamp = rospy.Time.now()
            accel_raw_y_msg.header.frame_id = "/base_link"
            accel_raw_y_msg.data = accel_raw['y']

            accel_raw_z_msg = Float64()
            accel_raw_z_msg.header.stamp = rospy.Time.now()
            accel_raw_z_msg.header.frame_id = "/base_link"
            accel_raw_z_msg.data = accel_raw['z']

            gyro_msg = Gyroscope()
            gyro_msg.header.stamp = rospy.Time.now()
            gyro_msg.header.frame_id = "/base_link"
            gyro_msg.x = gyro_only['pitch']
            gyro_msg.y = gyro_only['roll']
            gyro_msg.z = gyro_only['yaw']

            gyro_pitch_msg = Pitch()
            gyro_pitch_msg.header.stamp = rospy.Time.now()
            gyro_pitch_msg.header.frame_id = "/base_link"
            gyro_pitch_msg.data = gyro_only['pitch']

            gyro_roll_msg = Roll()
            gyro_roll_msg.header.stamp = rospy.Time.now()
            gyro_roll_msg.header.frame_id = "/base_link"
            gyro_roll_msg.data = gyro_only['roll']

            gyro_yaw_msg = Yaw()
            gyro_yaw_msg.header.stamp = rospy.Time.now()
            gyro_yaw_msg.header.frame_id = "/base_link"
            gyro_yaw_msg.data = gyro_only['yaw']

            gyro_raw_msg = Gyroscope()
            gyro_raw_msg.header.stamp = rospy.Time.now()
            gyro_raw_msg.header.frame_id = "/base_link"
            gyro_raw_msg.x = gyro_raw['x']
            gyro_raw_msg.y = gyro_raw['y']
            gyro_raw_msg.z = gyro_raw['z']

            gyro_raw_x_msg = Float64()
            gyro_raw_x_msg.header.stamp = rospy.Time.now()
            gyro_raw_x_msg.header.frame_id = "/base_link"
            gyro_raw_x_msg.data = gyro_raw['x']

            gyro_raw_y_msg = Float64()
            gyro_raw_y_msg.header.stamp = rospy.Time.now()
            gyro_raw_y_msg.header.frame_id = "/base_link"
            gyro_raw_y_msg.data = gyro_raw['y']

            gyro_raw_z_msg = Float64()
            gyro_raw_z_msg.header.stamp = rospy.Time.now()
            gyro_raw_z_msg.header.frame_id = "/base_link"
            gyro_raw_z_msg.data = gyro_raw['z']

            north_msg = Magnetometer()
            north_msg.header.stamp = rospy.Time.now()
            north_msg.header.frame_id = "/base_link"
            north_msg.north = north
            
            compass_msg = Orientation()
            compass_msg.header.stamp = rospy.Time.now()
            compass_msg.header.stamp = rospy.Time.now()
            compass_msg.x = compass['x']
            compass_msg.y = compass['y']
            compass_msg.z = compass['z']

            orientation_msg = Orientation()
            orientation_msg.header.stamp = rospy.Time.now()
            orientation_msg.header.stamp = rospy.Time.now()
            orientation_msg.x = orientation['pitch']
            orientation_msg.y = orientation['roll']
            orientation_msg.z = orientation['yaw']

            orientation_degree_msg = Orientation()
            orientation_degree_msg.header.stamp = rospy.Time.now()
            orientation_degree_msg.header.stamp = rospy.Time.now()
            orientation_degree_msg.x = orientation_deg['pitch']
            orientation_degree_msg.y = orientation_deg['roll']
            orientation_degree_msg.z = orientation_deg['yaw']

            orientation_rad_msg = Orientation()
            orientation_rad_msg.header.stamp = rospy.Time.now()
            orientation_rad_msg.header.stamp = rospy.Time.now()
            orientation_rad_msg.x = orientation_rad['pitch']
            orientation_rad_msg.y = orientation_rad['roll']
            orientation_rad_msg.z = orientation_rad['yaw']

            rospy.loginfo("imu/accelerometer: p: {pitch}, r: {roll}, y: {yaw}".format(**accel_only))
            rospy.loginfo("imu/accelerometer/raw: x: {x}, y: {y}, z: {z}".format(**accel_raw))
            rospy.loginfo("imu/gyroscope: p: {pitch}, r: {roll}, y: {yaw}".format(**gyro_only))
            rospy.loginfo("imu/gyroscope/raw: x: {x}, y: {y}, z: {z}".format(**gyro_raw))
            rospy.loginfo("imu/magnetometer: North: %s" % north)
            rospy.loginfo("imu/magnetometer/raw: x: {x}, y: {y}, z: {z}".format(**compass))
            rospy.loginfo("imu/orientation: p: {pitch}, r: {roll}, y: {yaw}".format(**orientation))
            rospy.loginfo("imu/orientation/degrees: p: {pitch}, r: {roll}, y: {yaw}".format(**orientation_deg))
            rospy.loginfo("imu/orientation/radians: p: {pitch}, r: {roll}, y: {yaw}".format(**orientation_rad))
            rospy.loginfo("imu/orientation/north: North: %s" % north)

            self.imuPub.publish(imu_msg)
            self.imuRawPub.publish(imu_raw_msg)
        
            self.accelerometerPub.publish(accel_msg)
            self.accelerometerPitchPub.publish(accel_pitch_msg)
            self.accelerometerRollPub.publish(accel_roll_msg)
            self.accelerometerYawPub.publish(accel_yaw_msg)
            self.accelerometerRawPub.publish(accel_raw_msg)
            self.accelerometerRawXPub.publish(accel_raw_x_msg)
            self.accelerometerRawYPub.publish(accel_raw_y_msg)
            self.accelerometerRawZPub.publish(accel_raw_z_msg)

            self.gyroscopePub.publish(gyro_msg)
            self.gyroscopePitchPub.publish(gyro_pitch_msg)
            self.gyroscopeRollPub.publish(gyro_roll_msg)
            self.gyroscopeYawPub.publish(gyro_yaw_msg)
            self.gyroscopeRawPub.publish(gyro_raw_msg)
            self.gyroscopeRawXPub.publish(gyro_raw_x_msg)
            self.gyroscopeRawYPub.publish(gyro_raw_y_msg)
            self.gyroscopeRawZPub.publish(gyro_raw_z_msg)

            self.magnetometerPub.publish(north_msg)
            self.magnetometerRawPub.publish(compass_msg)

            self.orientationPub.publish(orientation_msg)
            self.orientationDegreePub.publish(orientation_degree_msg)
            self.orientationRadiansPub.publish(orientation_rad_msg)
            self.orientationNorthPub.publish(north_msg)

            self.rate.sleep()

    def stop(self):
        self.enable = False

        self.imuPub.unregister()
        self.imuRawPub.unregister()
    
        self.accelerometerPub.unregister()
        self.accelerometerPitchPub.unregister()
        self.accelerometerRollPub.unregister()
        self.accelerometerYawPub.unregister()
        self.accelerometerRawPub.unregister()
        self.accelerometerRawXPub.unregister()
        self.accelerometerRawYPub.unregister()
        self.accelerometerRawZPub.unregister()

        self.gyroscopePub.unregister()
        self.gyroscopePitchPub.unregister()
        self.gyroscopeRollPub.unregister()
        self.gyroscopeYawPub.unregister()
        self.gyroscopeRawPub.unregister()
        self.gyroscopeRawXPub.unregister()
        self.gyroscopeRawYPub.unregister()
        self.gyroscopeRawZPub.unregister()

        self.magnetometerPub.unregister()
        self.magnetometerRawPub.unregister()

        self.orientationPub.unregister()
        self.orientationDegreePub.unregister()
        self.orientationRadiansPub.unregister()
        self.orientationNorthPub.unregister()

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    node_name = re.sub("-", "_", socket.gethostname()) + "_IMU"
    rospy.init_node(node_name, anonymous=False)
    # Go to class functions that do all the heavy lifting.

    imu = IMU()

    try:
        imu.start()
    except rospy.ROSInterruptException:
        imu.stop()
        pass
    # Allow ROS to go to all callbacks.
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()