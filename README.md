# robotcar-pkg

The robotcar-pkg is used for sending informations to and getting informations from the [RobotCar](https://github.com/Michdo93/robotcar). At first you have to make sure that the roscore is running. For controlling the RobotCar the [robotcar-controller](https://github.com/Michdo93/robotcar_controller) is needed. To receive informations from actors or sensors the [robotcar_subscriber](https://github.com/Michdo93/robotcar_subscriber) or the [robotcar_sensorfusion_examples](https://github.com/Michdo93/robotcar_sensorfusion_examples) package could be used.

The ROS Nodes which uses the Publisher-Subscriber-Patter are using drivers, librares and configurations from the [robotcar](https://github.com/Michdo93/robotcar) to operate its actors and sensors.

Therefore it uses the [std_msgs](http://docs.ros.org/en/melodic/api/std_msgs/html/index-msg.html), the [sensor_msgs](http://docs.ros.org/en/api/sensor_msgs/html/index-msg.html) and the [robotcar_msgs](https://github.com/Michdo93/robotcar_msgs).

## How to Use

As recommended in the [ROS Tutorials](https://wiki.ros.org/ROS/Tutorials) the control computer respectively operating computer should run the roscore. So at first open a terminal window and execute `roscore`. After that open a second terminal window and run `roslaunch robotcar robotcar.launch`. (If not you can start each node manually).

The RobotCar uses its `hostname` as first variable for the topic adresses. So as example you can subscribe from `robotcar/raspicam/image/compressed` a [CompressedImage Message](http://docs.ros.org/en/api/sensor_msgs/html/msg/CompressedImage.html) if the `hostname` of the RobotCar was `robotcar`. Fpr renaming the hostname you can use following [link](https://thepihut.com/blogs/raspberry-pi-tutorials/19668676-renaming-your-raspberry-pi-the-hostname).

## Launch Files

The robotcar-pkg provides following two launch files:

|    Launch Files      |                 Subscription                |
|--------------------- | --------------------------------------------|
| robotcar.launch      | Starts all nodes of the RobotCar            |
| ultrasonic.launch    | Starts all ultrasonic nodes of the RobotCar |

If further developments creates newer nodes you should edit the robotcar.launch file. If you want to test only a few nodes which as example are used for an ADAS you can [create a new launch file](http://wiki.ros.org/roslaunch/XML).

Recommendation: The robotcar.launch should be enabled with a [services file for systemd](https://www.raspberrypi.org/documentation/linux/usage/systemd.md) so that all nodes would be available after the startup of the RobotCar.

## CameraMovement Node

Currently not finished!

(You can run it manually with `rosrun robotcar cameraMovement.py`)

## Control Node

It subcribes from the [RobotCarController Node](https://github.com/Michdo93/robotcar_controller).

|                 Topic Address                |            Message Type       |
|--------------------------------------------- | ------------------------------|
|robot_host + /control/speed                   | [std_msgs/Float64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html)          |
|robot_host + /control/steer                   | [std_msgs/Float64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html)          |

Also it publishes to the Motor Node and to the Steer Node to produce the driving process.

|                 Topic Address                |            Message Type       |
|--------------------------------------------- | ------------------------------|
| hostname + /motor/set                        | [robotcar_msgs/Motor](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Motor.md)          |
| hostname + /steer/set                        | [robotcar_msgs/Steer](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Steer.md)          |

You can run it manually with `rosrun robotcar control.py`

## FrontInfrared Node

It publishes informations from the front infrared sensor. The IRGP2Y0A02YKOF driver from the [robotcar](https://github.com/Michdo93/robotcar) and [Adafruit Python MCP3008](https://github.com/adafruit/Adafruit_Python_MCP3008) is used. A system wide installation guide you can found at the [RobotCar](https://github.com/Michdo93/robotcar).

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
| hostname + /infrared/front/distance                   | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
| hostname + /infrared/front/relative_velocity          | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

You can run it manually with `rosrun robotcar frontInfrared.py`

## FrontLeftUltrasonic Node

It publishes informations from the left front ultrasonic sensor. The UltrasonicHCSR04 driver from the [robotcar](https://github.com/Michdo93/robotcar) is used.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
| hostname + /ultrasonic/front/left/distance            | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
| hostname + /ultrasonic/front/left/relative_velocity   | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

You can run it manually with `rosrun robotcar frontLeftUltrasonic.py`

## FrontRightUltrasonic Node

It publishes informations from the right front ultrasonic sensor. The UltrasonicHCSR04 driver from the [robotcar](https://github.com/Michdo93/robotcar) is used.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
| hostname + /ultrasonic/front/right/distance           | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
| hostname + /ultrasonic/front/right/relative_velocity  | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

You can run it manually with `rosrun robotcar frontRightUltrasonic.py`

## FrontTimeOfFlight Node

It publishes informations from the front time-of-flight sensor. The ToFVL53L1X driver from the [robotcar](https://github.com/Michdo93/robotcar) and [vl53l1x-python library](https://github.com/pimoroni/vl53l1x-python) is used. A system wide installation guide you can found at the [RobotCar](https://github.com/Michdo93/robotcar).

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
| hostname + /time_of_flight/front/distance             | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
| hostname + /time_of_flight/front/relative_velocity    | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

You can run it manually with `rosrun robotcar frontTimeOfFlight.py`

## FrontUltrasonic Node

It publishes informations from the front ultrasonic sensor. The UltrasonicParallax driver from the [robotcar](https://github.com/Michdo93/robotcar) is used.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
| hostname + /ultrasonic/front/distance                 | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
| hostname + /ultrasonic/front/relative_velocity        | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

You can run it manually with `rosrun robotcar frontUltrasonic.py`

## GPS Node

Neither created nor finished yet because the GPS module seems not working.

## Imu Node

It publishes informations from the Imu of the Sense HAT. The [Sense HAT Library](https://github.com/astro-pi/python-sense-hat) is needed. Further informations you can get from the [Sense HAT API](https://pythonhosted.org/sense-hat/api/). A system wide installation guide you can found at the [RobotCar](https://github.com/Michdo93/robotcar).

|                       Topic Address                        |             Message Type        |
|----------------------------------------------------------- | --------------------------------|
| hostname + /imu                                            | [sensor_msgs/Imu](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html)                |
| hostname + /imu/raw                                        | [sensor_msgs/Imu](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html)                |
| hostname + /imu/accelerometer                              | [robotcar_msgs/Accelerometer](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Accelerometer.md)    |
| hostname + /imu/accelerometer/pitch                        | [robotcar_msgs/Pitch](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Pitch.md)    |
| hostname + /imu/accelerometer/roll                         | [robotcar_msgs/Roll](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Roll.md)     |
| hostname + /imu/accelerometer/yaw                          | [robotcar_msgs/Yaw](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Yaw.md)      |
| hostname + /imu/accelerometer/raw                          | [robotcar_msgs/Accelerometer](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Accelerometer.md)    |
| hostname + /imu/accelerometer/raw/x                        | [std_msgs/Float64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html)     |
| hostname + /imu/accelerometer/raw/y                        | [std_msgs/Float64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html)     |
| hostname + /imu/accelerometer/raw/z                        | [std_msgs/Float64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html)     |
| hostname + /imu/gyroscope                            | [robotcar_msgs/Gyroscope](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Gyroscope.md)        |
| hostname + /imu/gyroscope/pitch                            | [robotcar_msgs/Pitch](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Pitch.md)        |
| hostname + /imu/gyroscope/roll                             | [robotcar_msgs/Roll](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Roll.md)         |
| hostname + /imu/gyroscope/yaw                              | [robotcar_msgs/Yaw](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Yaw.md)          |
| hostname + /imu/gyroscope/raw                        | [robotcar_msgs/Gyroscope](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Gyroscope.md)        |
| hostname + /imu/gyroscope/raw/x                            | [std_msgs/Float64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html)         |
| hostname + /imu/gyroscope/raw/y                            | [std_msgs/Float64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html)         |
| hostname + /imu/gyroscope/raw/z                            | [std_msgs/Float64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html)         |
| hostname + /imu/magenetometer                        | [robotcar_msgs/Magnetometer](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Magnetometer.md)     |
| hostname + /imu/magenetometer/raw                    | [robotcar_msgs/Orientation](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Orientation.md)      |
| hostname + /imu/orientation                          | [robotcar_msgs/Orientation](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Orientation.md)      |
| hostname + /imu/orientation/degrees                  | [robotcar_msgs/Orientation](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Orientation.md)      |
| hostname + /imu/orientation/radians                  | [robotcar_msgs/Orientation](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Orientation.md)      |
| hostname + /imu/orientation/north                    | [robotcar_msgs/Magnetometer](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Magnetometer.md)     |
        
You can run it manually with `rosrun robotcar imu.py`

## Meteorological Node

It publishes informations from the meteorological sensors of the Sense HAT. The [Sense HAT Library](https://github.com/astro-pi/python-sense-hat) is needed. Further informations you can get from the [Sense HAT API](https://pythonhosted.org/sense-hat/api/). A system wide installation guide you can found at the [RobotCar](https://github.com/Michdo93/robotcar).

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
| hostname + /meteorological/pressure                   | [sensor_msgs/FluidPressure](http://docs.ros.org/en/api/sensor_msgs/html/msg/FluidPressure.html)     |
| hostname + /meteorological/humidity                   | [sensor_msgs/RelativeHumidity](http://docs.ros.org/en/api/sensor_msgs/html/msg/RelativeHumidity.html)  |
| hostname + /meteorological/temperature                | [sensor_msgs/Temperature](http://docs.ros.org/en/api/sensor_msgs/html/msg/Temperature.html)       |

You can run it with `rosrun robotcar meteorological.py`

## Motor Node

It subscribes informations from the [Controller](https://github.com/Michdo93/robotcar_controller) or an ADAS so that the motor can produce a desired driving process. The JGA25_370 driver from the [robotcar](https://github.com/Michdo93/robotcar) is used.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
| hostname + /motor/set                                | [robotcar_msgs/Motor](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Motor.md)            |
| hostname + /motor/set/rate                                | [std_msgs/Int8](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Int8.html)            |
| hostname + /motor/set/direction                                | [std_msgs/String](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html)            |
| hostname + /motor/set/ms                             | [robotcar_msgs/Velocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Velocity.md)         |
| hostname + /motor/set/kmh                            | [robotcar_msgs/Velocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Velocity.md)         |
| hostname + /motor/set/mph                            | [robotcar_msgs/Velocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Velocity.md)         |
| hostname + /motor/engine/stop                                | [std_msgs/Bool](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Bool.html)            |

Also it publishes informations from the motor. In addition to sensor information, an ADAS probably also needs this informations.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
| hostname + /motor/get                                | [robotcar_msgs/Motor](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Motor.md)            |
| hostname + /motor/get/rate                                | [std_msgs/Int8](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Int8.html)            |
| hostname + /motor/get/direction                                | [std_msgs/String](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html)            |
| hostname + /motor/get/ms                             | [robotcar_msgs/Velocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Velocity.md)         |
| hostname + /motor/get/kmh                            | [robotcar_msgs/Velocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Velocity.md)         |
| hostname + /motor/get/mph                            | [robotcar_msgs/Velocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Velocity.md)         |
| hostname + /motor/get/ms/max                             | [robotcar_msgs/Velocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Velocity.md)         |
| hostname + /motor/get/kmh/max                            | [robotcar_msgs/Velocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Velocity.md)         |
| hostname + /motor/get/mph/max                            | [robotcar_msgs/Velocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Velocity.md)         |
| hostname + /motor/get/dc_max                                | [std_msgs/Int8](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Int8.html)            |
| hostname + /motor/get/duty_cycle                            | [robotcar_msgs/DutyCycle](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/DutyCycle.md)        |
| hostname + /motor/get/duty_cycle/frequency                            | [robotcar_msgs/DutyCycle](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/DutyCycle.md)        |
| hostname + /motor/get/pwm                            | [robotcar_msgs/DutyCycle](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/DutyCycle.md)        |
| hostname + /motor/get/rpm                            | [robotcar_msgs/Rpm](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Rpm.md)              |
| hostname + /motor/get/rpm/current                            | [robotcar_msgs/Rpm](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Rpm.md)              |
| hostname + /motor/get/rpm/max                            | [robotcar_msgs/Rpm](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Rpm.md)              |
        
You can run it manually with `rosrun robotcar motor.py`

## Radar Node

Warning: The radar sensors a not working reliable! A use of the Radar Node is not recommend because the relative velocity is inaccurate! The CDM324, HB100 or IPM165 driver from the [robotcar](https://github.com/Michdo93/robotcar) could be used. It publishes informations from the radar sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
| hostname + /radar                                     | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|
| hostname + /radar/relative_velocity                   | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

You can run it manually with `rosrun robotcar radar.py`

## RearInfrared Node

It publishes informations from the rear infrared sensor. The IRGP2Y0A02YKOF driver from the [robotcar](https://github.com/Michdo93/robotcar) and [Adafruit Python MCP3008](https://github.com/adafruit/Adafruit_Python_MCP3008) is used. A system wide installation guide you can found at the [RobotCar](https://github.com/Michdo93/robotcar).

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
| hostname + /infrared/rear/distance                    | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
| hostname + /infrared/rear/relative_velocity           | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

You can run it manually with `rosrun robotcar rearInfrared.py`

## RearLeftUltrasonic Node

It publishes informations from the left rear ultrasonic sensor. The UltrasonicHCSR04 driver from the [robotcar](https://github.com/Michdo93/robotcar) is used.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
| hostname + /ultrasonic/rear/left/distance             | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
| hostname + /ultrasonic/rear/left/relative_velocity    | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

You can run it manually with `rosrun robotcar rearLeftUltrasonic.py`

## RearRightUltrasonic Node

It publishes informations from the right rear ultrasonic sensor. The UltrasonicHCSR04 driver from the [robotcar](https://github.com/Michdo93/robotcar) is used.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
| hostname + /ultrasonic/rear/right/distance            | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
| hostname + /ultrasonic/rear/right/relative_velocity   | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

You can run it manually with `rosrun robotcar rearRightUltrasonic.py`

## RearTimeOfFlight Node

It publishes informations from the rear time-of-flight sensor. The ToFVL53L1X driver from the [robotcar](https://github.com/Michdo93/robotcar) and [vl53l1x-python library](https://github.com/pimoroni/vl53l1x-python) is used. A system wide installation guide you can found at the [RobotCar](https://github.com/Michdo93/robotcar).

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
| hostname + /time_of_flight/rear/distance              | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
| hostname + /time_of_flight/rear/relative_velocity     | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

You can run it manually with `rosrun robotcar rearTimeOfFlight.py`

## RearUltrasonic Node

It publishes informations from the rear ultrasonic sensor. The UltrasonicParallax driver from the [robotcar](https://github.com/Michdo93/robotcar) is used.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
| hostname + /ultrasonic/rear/distance                  | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |
| hostname + /ultrasonic/rear/relative_velocity         | [robotcar_msgs/RelativeVelocity](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/RelativeVelocity.md)|

You can run it manually with `rosrun robotcar rearUltrasonic.py`

## Steer Node

It subscribes informations from the [Controller](https://github.com/Michdo93/robotcar_controller) or an ADAS so that the steer servo motor can produce a desired driving process. The ServoMotor driver from the [robotcar](https://github.com/Michdo93/robotcar) and the Adafruit_Python_PCA9685-master library is used. A system wide installation guide you can found at the [RobotCar](https://github.com/Michdo93/robotcar).

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
| hostname + /steer/set                                | [robotcar_msgs/Steer](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Steer.md)            |
| hostname + /steer/set/pwm                                | [robotcar_msgs/Steer](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Steer.md)            |
| hostname + /steer/set/degree                                | [robotcar_msgs/ServoDeg](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/ServoDeg.md)         |
| hostname + /steer/reset                                | [std_msgs/Bool](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Bool.html)            |

Also it publishes informations from the motor. In addition to sensor information, an ADAS probably also needs this informations.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
| hostname + /steer/get                                | [robotcar_msgs/Steer](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Steer.md)            |
| hostname + /steer/get/pwm                                | [robotcar_msgs/Steer](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Steer.md)            |
| hostname + /steer/get/degree                                | [robotcar_msgs/ServoDeg](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/ServoDeg.md)         |
| hostname + /steer/get/intervall                      | [robotcar_msgs/ServoIntervall](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/ServoIntervall.md)   |
| hostname + /steer/get/intervall/pwm                      | [robotcar_msgs/ServoIntervall](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/ServoIntervall.md)   |
| hostname + /steer/get/intervall/degree               | [robotcar_msgs/ServoIntervallDeg](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/ServoIntervallDeg.md)|
| hostname + /steer/get/channel               | [std_msgs/Int8](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Int8.html)            |
| hostname + /steer/get/min                            | [robotcar_msgs/Steer](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Steer.md)            |
| hostname + /steer/get/neutral                        | [robotcar_msgs/Steer](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Steer.md)            |
| hostname + /steer/get/max                            | [robotcar_msgs/Steer](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Steer.md)            |
| hostname + /steer/get/min/pwm                            | [robotcar_msgs/Steer](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Steer.md)            |
| hostname + /steer/get/neutral/pwm                        | [robotcar_msgs/Steer](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Steer.md)            |
| hostname + /steer/get/max/pwm                            | [robotcar_msgs/Steer](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Steer.md)            |
| hostname + /steer/get/min/degree                     | [robotcar_msgs/ServoDeg](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/ServoDeg.md)         |
| hostname + /steer/get/neutral/degree                 | [robotcar_msgs/ServoDeg](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/ServoDeg.md)         |
|robot_host + /steer/get/max/degree                     | [robotcar_msgs/ServoDeg](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/ServoDeg.md)         |
| hostname + /steer/get/range                          | [robotcar_msgs/ServoRange](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/ServoRange.md)       |
| hostname + /steer/get/range/pwm                          | [robotcar_msgs/ServoRange](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/ServoRange.md)       |
| hostname + /steer/get/range/degree                   | [robotcar_msgs/ServoRangeDeg](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/ServoRangeDeg.md)    |
        
You can run it manually with `rosrun robotcar steer.py`
