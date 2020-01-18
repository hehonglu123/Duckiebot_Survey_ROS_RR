# Duckiebot ROS and RR Survey
## Introduction
In this survey, we want students to learn and compare robotics middleware: widely used [Robot Operating System](http://wiki.ros.org/) (ROS) and [Robot Raconteur](https://www.robotraconteur.com/) (RR) designed by RPI alumni. Given a duckiebot with [Ubuntu Mate (18.04)](https://ubuntu-mate.org/blog/ubuntu-mate-bionic-final-release/) image, the goal is to use ROS and Robot Raconteur to achieve lane following.
## Duckiebot
[Duckiebot](https://www.duckietown.org/) is a wheeled robot with 2 motors, one Raspberry Pi, a motor HAT and a Pi Camera. You have the control over 2 motor wheels and read image or video from Pi Cam. Please follow Duckiebot Setup Guide before proceeding. To access the duckiebot, use **ssh** command with given *username*, *password* and *IP address*:
```
ssh <username>@<IP>
```
## Robot Raconteur Tutorial
Robot Raconteur is an object oriented Service-Client middleware. An RR service generally runs with a sensor/robot to have communication directly with them. An RR client usually can receive sensor messages from service and call object function to command the robot. Take a look into given webcam examples on https://github.com/robotraconteur/RobotRaconteur_Python_Examples. Try connecting a webcam to your Ubuntu laptop, running SimpleWebcamService.py and SimpleWebcamClient_streaming.py to get a live window streaming your webcam. 
To get a good understanding of Robot Raconteur, we’ll demonstrate how RR works on duckiebot with keyboard teleop. 
### RR Service:
The RR service for duckiebot is to control both motors. Inside `Duckiebot_Survey/RobotRaconteur/DuckiebotRR-Service-Drive.py`, there is RR robdef called `drive_servicedef`
```
drive_servicedef="""
	#Service to provide sample interface to the Duckiebot Drive
	service experimental.duckiebot
	
	stdver 0.9
	
	object Drive
	    property int8 LEFT_MOTOR_MIN_PWM 
	    property int8 LEFT_MOTOR_MAX_PWM 
	    property int8 RIGHT_MOTOR_MIN_PWM
	    property int8 RIGHT_MOTOR_MAX_PWM 
	    property int8 SPEED_TOLERANCE 
	
	    function void setWheelsSpeed(double v_left, double v_right)
	
	end object
	"""
```
  You could consider this as a python class object declaration with variables and functions, but it’s also necessary to create an actual python class object including those variables/functions or some others that you don’t need on client side. In short, variables and functions inside **robdef** are the ones you have access to on client side. Inside class object **DaguWheelsDriver**, the PWM output is specified here to control the motor speed. Note that the motor doesn’t have a wheel encoder, so the values here doesn’t mean the actual velocity of the wheel speed. 
  At the bottom of the file, which is the main part for Robot Raconteur, we have `with RR.ServerNodeSetup("Drive_Service",2356) as node_setup:`
`Drive_Service` here is the node name, and `2356` is the port for TCP communication. The object is initialized by `obj=DaguWheelsDriver()`
The major difference for Robot Raconteur is that it has security over service. The password is hashed and a username is also required to connect to the service. 
`authdata="cats be7af03a538bf30343a501cb1c8237a0 objectlock"`
Here the username is **cats** and password is **cats111!**. 
To expose the service over network, the service is registered through
```
RRN.RegisterServiceType(drive_servicedef)
RRN.RegisterService("Drive","experimental.duckiebot.Drive",obj,security)
```
So in this case the service name is called `Drive`, and the object type, the actual object and security is exposed together with the service.
`raw_input()` is just a function to hold the service run indefinitely, and if the user wants to terminate the service, just press `Enter` key and the service will shutdown. In order to use Robot Raconteur library as RR service, it’s necessary to import RR library at start:
```
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
```
### RR Client
The example for RR client is duckiebot keyboard control. Pygame is used as a virtual joystick here, and to instally pygame, simply type `$ pip install pygame`.  Inside `Duckiebot_Survey/RobotRaconteur/Keyboard_Teleop/keyboard.py`, the major part is pygame visualization. At the bottom part of this script, there is
```
url='rr+tcp://<hostname>:2356?service=Drive'
c=RRN.ConnectService(url,"cats",{"password":RR.RobotRaconteurVarValue("cats111!","string")})
```
The url is the IP address of duckiebot, the port the service is on and the service name. Replace `<hostname>` with the duckiebot hostname or IP address. In the argument of **ConnectService**, we also specify the username and password to connect to the service. The return variable for this function is the object created in service, so you can simply modify the duckiebot wheel speed by calling `c.setWheelsSpeed(0.5,0.5)`. And this is demonstrated in each key press inside the `loop()` function. In order to use Robot Raconteur library as RR service, it’s necessary to import RR client library at start: 
```
from RobotRaconteur.Client import *
```
### Running RR
Once the scripts are ready to run, simply run it as a python script, and it’s necessary to start the service first and then client.
```
python DuckiebotRR-Service-Drive.py 	#on duckiebot
python keyboard.py		    	#on laptop
```
### Task
You are provided with `Duckiebot-RR-Service-Drive.py` and `Duckiebot-RR-Service-PiCam.py`, and the goal is to make the duckiebot do lane following. All the scripts should be running on the duckiebot side. The usage of `Duckiebot-RR-Service-PiCam.py` is similar to the given example `SimpleWebcamService.py`. The task file is called `DuckiebotRR-Client-LaneFollower.py`, and fill in the `#TO DO` sections. You can either edit the file on duckiebot directly using `nano` or `vim`, or you can modify the file on laptop and use `scp` command to copy the file onto duckiebot.

## ROS Tutorial
The structure of ROS is a little different from Robot Raconteur. First it has the Publisher-Subscriber relationship between different nodes. In our case the subscriber is on the duckiebot, listening to the speed command messages from remote Ubuntu laptop. And obviously the Ubuntu laptop is the publisher, so that user can publish command toward the duckiebot. Another relationship in ROS is Master-Slave. In order to use ROS in python, it’s necessary to `import rospy` at the start of each script.

### ROS Master
To initiate a ROS communication from laptop to the duckiebot, it’s necessary to identify which one is ROS_MASTER. This needs to be done on both side by 
`$ export ROS_MASTER_URI=http://<hostname>:11311`
, where the hostname is the laptop’s hostname or IP address. Once this is done, you can look up this value by 
`$ echo $ROS_MASTER_URI`
to make sure it’s set. Note you need to do this for every new terminal opened, and you only need one roscore running in one Master-Slave setup, which should be on the laptop side. So for the keyboard control example, open up three terminals, with one `ssh` into the duckiebot, and type in above command in all three terminals. Start a **roscore** in one terminal by typing `$ roscore`.
### ROS Subscriber
The ROS script on the duckiebot contains a subscriber for motor command and a publisher for image acquisition. The motor command subscriber is `Duckiebot_Survey/catkin_ws/src/motor_control/src/motor_control.py` on the duckiebot side. This script is looks very similar to RR Drive Service because most part is the provided python class object. Inside the subscriber, there’s a function `listener()`, and this is the main part for ROS subscriber. 
```
rospy.init_node('motor_control', anonymous=True)
rospy.Subscriber("motor_command", Twist, callback)
```
Above lines initialize the ROS node name as *motor_control*, and subscribe to **ROS Topic** *motor_command*, with [geometry_msgs/Twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html) type of [ROS message](http://wiki.ros.org/msg). The `callback()` function controls the motor based on messages received. And `rospy.spin()` makes the subscriber runs indefinitely. To run this subscriber, simply type in 
```
$ python motor_control.py
```

### ROS Publisher
The example for ROS publisher on laptop side is to send motor command over to the subscriber on the duckiebot side. Inside `Duckiebot_Survey/catkin_ws/src/motor_control/src/keyboard.py` on laptop side, it’s again similar to the RR client. At the bottom part of this script, 
```
pub = rospy.Publisher('motor_command', Twist, queue_size=0)
```
This line initialize the ROS publisher, publishing to **ROS Topic** *motor_command*, with [geometry_msgs/Twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html) type of [ROS message](http://wiki.ros.org/msg). The queue_size is the buffer size for publishing. 
```
rospy.init_node('motor_command', anonymous=True)
```
This line is the same as the one in subscriber, which initialize the ROS node named *motor_command*.
```
rate = rospy.Rate(10)
```
And here the rate specifies the publishing rate.
Inside the `loop()` function is where the `Twist()` message is composed. 
```
velocity = Twist()
velocity.linear.x=0
velocity.linear.y=0
```
The `velocity.linear.x` and `velocity.linear.y` corresponds to left and right wheel velocity respectively. And each loop the composed message is published out by 
```
pub.publish(velocity)
```
To run this publisher, simply type 
`$ python keyboard.py` 
in the terminal. Try playing with the keyboard control and see how messages are published and subscribed between laptop and duckiebot.

### Picam Node ROS
You are also provided with a ROS node for image publishing, forked from https://github.com/UbiquityRobotics/raspicam_node. To get it running, first run 
```
$ sudo apt install ros-melodic-raspicam-node
```
Then add the following line 
```
yaml https://raw.githubusercontent.com/UbiquityRobotics/rosdep/master/raspberry-pi.yaml
```
to the file `/etc/ros/rosdep/sources.list.d/30-ubiquity.list`.
Then run 
```
$ rosdep update
$ cd ~/Duckiebot_Survey/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=melodic -y –skip-keys libraspberrypi0
```
The rostopic name should be *raspicam_node/image* and the message type is [sensor_msgs/Image.msg](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html). Remember you can always find current topic by 
```
$ rostopic list
```
### Running ROS
Once you have all scripts ready, build all of them by 
```
$ cd ~/Duckiebot_Survey/catkin_ws
$ catkin_make
$ source devel/setup.bash
```
For rospy scripts, simply running it in python works with a **roscore** on. For roscpp scripts like Picam node, there’s usually a launch file to bring everything up:
```
$ roslaunch raspicam_node camerav2_640x480.launch enable_raw:=true
```
Note that the command roslaunch will bring up a **roscore**, so you could launch Picam first and then run other rospy scripts. To stop a script, simple press `ctrl+c`.
### Task
You are provided with ROS subscriber motor_control.py for motor command and ROS publisher for image publishing. Try to make the duckiebot do lane following in scripts `~/Duckiebot_Survey/catkin_ws/src/lane_following/src/lane_following.py` by filling in the `#TO DO` sections. The main part is to complete the publisher for motor command and subsciber for image. You can either edit the file on duckiebot directly using `nano` or `vim`, or you can modify the file on laptop and use `scp` command to copy the file onto duckiebot. 

To test the scripts, open up three terminals and all `ssh` into the duckiebot. Go to `~/Duckiebot_Survey/catkin_ws` and run
```
$ roslaunch raspicam_node camerav2_640x480.launch enable_raw:=true
$ python src/motor_control/src/motor_control.py
$ python src/lane_following/src/lane_following.py

