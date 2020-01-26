# Duckiebot ROS and RR Survey
## Introduction
In this survey, we want students to learn and compare robotics middleware: widely used [Robot Operating System](http://wiki.ros.org/) (ROS) and [Robot Raconteur](https://www.robotraconteur.com/) (RR) designed by RPI alumni. Given a duckiebot with [Ubuntu Mate (18.04)](https://ubuntu-mate.org/blog/ubuntu-mate-bionic-final-release/) image, the goal is to use ROS and Robot Raconteur to achieve lane following.
On the computer/duckiebot, please follow instructions on [RR website](https://github.com/robotraconteur/robotraconteur/wiki/Download)(Ubuntu Xenial python2 version) and [ROS website](http://wiki.ros.org/melodic/Installation/Ubuntu)(Desktop version) to install both of them.
## Duckiebot
[Duckiebot](https://www.duckietown.org/) is a wheeled robot with 2 motors, one Raspberry Pi, a motor HAT and a Pi Camera. You have the control over 2 motor wheels and read image or video from Pi Cam. Please follow Duckiebot Setup Guide before proceeding. To access the duckiebot, use **ssh** command with given *username*, *password* and *IP address*:
```
ssh <username>@<IP>
```
First, clone this repository to the root directory on duckiebot and computer
```
cd ~
git clone https://github.com/hehonglu123/Duckiebot_Survey_ROS_RR.git
```

## Robot Raconteur Survey
Robot Raconteur is an object oriented Service-Client middleware. An RR service generally runs with a sensor/robot to have communication directly with them. An RR client usually can receive sensor messages from service and call object function to command the robot. In this survey, we’ll first demonstrate how RR works with a webcam streaming. 
### RR Service:
Inside `Duckiebot_Survey_ROS_RR/RobotRaconteur`, there is RR robdef called `experimental.createwebcam2.robdef`
`service experimental.createwebcam2` defines the service robdef name, and it'll be refered in RR service script as this file. `stdver 0.9` is current RR version. 
```
struct WebcamImage
    field int32 width
    field int32 height
    field int32 step
    field uint8[] data
end
```
This is a sample structure data type in RR, similar to a python object. You can look for other data type in [RR python documentation](https://s3.amazonaws.com/robotraconteurpublicfiles/docs/IntroductionToRobotRaconteur-2019-06-19.pdf).
```
object Webcam
    property string Name [readonly]
    function WebcamImage CaptureFrame()

    function void StartStreaming()
    function void StopStreaming()
    pipe WebcamImage FrameStream [readonly]

    function WebcamImage_size CaptureFrameToBuffer()
    memory uint8[] buffer [readonly]
    memory uint8[*] multidimbuffer [readonly]

end
```
This object Webcam is a sample RR object that could be passed from service to client. Inside each object there could be variables and functions that you can look up or call directly inside client.
```
object WebcamHost
    property string{int32} WebcamNames [readonly]
    objref Webcam{int32} Webcams
end
```
This object WebcamHost is the object that is actually passed in this example. It includes a list of webcam names and webcam objects, incase when there're multiple webcams connected.

Now take a look at the actual service script `SimpleWebcamService.py `, at the very start, we import RR library:
```
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
```
Right after that, `class Webcam_impl(object)` refers to the RR object `Webcam`, and `class WebcamHost_impl(object)` refers to the RR object `WebcamHost`. Inside `main()` function, the object is created by 
```
obj=WebcamHost_impl(camera_names)
```
RR service is registered, and pass that object to the network by
```
RRN.RegisterServiceTypeFromFile("experimental.createwebcam2")
RRN.RegisterService("Webcam","experimental.createwebcam2.WebcamHost",obj)
```

### RR Client:
As for the RR client `SimpleWebcamClient_streaming.py`, the first step is also to import RR client library:
```
from RobotRaconteur.Client import *
```
Inside `main()` function, it's necessary to specify the IP and port of the RR service, and it's done by
```
url='rr+tcp://<hostname>:<port>?service=Webcam'  
c_host=RRN.ConnectService(url)
```
It's necessary to replace <hostname> and <port> with the actual ones the RR service is using (<hostname> can be replaced by IP address as well). Notice the variable `c_host` is of type `WebcamHost` in RR robdef, and the actual `Webcam` object is retrieved by
```
c=c_host.get_Webcams(0)
```
The line `p.PacketReceivedEvent+=new_frame` triggers the callback function `def new_frame(pipe_ep)`, which updates the global variable `current_frame` continuously. `c.StartStreaming()` starts the streaming process so the client receives the real-time image frame.

### Running RR
Both Webcam service and client are ready to run, simply run it as a python script in a terminal, and it’s necessary to start the service first and then client. So go to `~/Duckiebot_Survey_ROS_RR/RobotRaconteur/` first, and run
```
python SimpleWebcamService.py 	#on duckiebot
python SimpleWebcamClient_streaming.py	#on laptop
```
### Task 1
Given above examples for webcam service and client, write RR service and client for the Picam on duckiebot, so that on the computer side you can get video streaming from the Picam. Picam python package is already installed, and their API is listed here: https://picamera.readthedocs.io/en/release-1.13/api_streams.html.


### Task 2
Inside `~/Duckiebot_Survey_ROS_RR/RobotRaconteur/`, there's a scirpt called `Example_Drive.py`. This script can run directly, and makes the motor drive straight for 5 seconds. The motor drivers are located in the same directory, and the task is to fill in `#TODO` section to make this an RR motor drive service. After that, try create an RR client script on your laptop to drive the duckiebot remotely.


You are provided with `DuckiebotRR-Service-Drive.py` and `DuckiebotRR-Service-PiCam.py`, and the goal is to make the duckiebot do lane following. The usage of `DuckiebotRR-Service-PiCam.py` is similar to the given example `SimpleWebcamService.py`. The task file is called `DuckiebotRR-Client-LaneFollower.py`, and fill in the `#TO DO` sections (search `TO DO` by `ctrl+F`). Both  `DuckiebotRR-Service-Drive.py` and `DuckiebotRR-Service-PiCam.py` should be running on the duckiebot, and `DuckiebotRR-Client-LaneFollower.py` should be running on the computer side. You can either edit the file on duckiebot directly using `nano` or `vim`, or you can modify the file on laptop and use `scp` command to copy the file onto duckiebot.

## ROS Tutorial
The structure of ROS is a little different from Robot Raconteur. First it has the Publisher-Subscriber relationship between different nodes. In our case the subscriber is on the duckiebot, listening to the speed command messages from remote Ubuntu laptop. And obviously the Ubuntu laptop is the publisher, so that user can publish command toward the duckiebot. Another relationship in ROS is Master-Slave. In order to use ROS in python, it’s necessary to `import rospy` at the start of each script.

### ROS Master
To initiate a ROS communication from laptop to the duckiebot, it’s necessary to identify which one is ROS_MASTER. This needs to be done on both side by 
`$ export ROS_MASTER_URI=http://<hostname>:11311`
, where the hostname is the laptop’s hostname or IP address. Once this is done, you can look up this value by 
`$ echo $ROS_MASTER_URI`
to make sure it’s set. Note you need to do this for every new terminal opened, and you only need one roscore running in one Master-Slave setup, which should be on the laptop side. This only needs to be done if you need ROS communication among different machines.
So for the keyboard control example, we have communication between duckiebot and laptop. On the laptop side, open up three terminals, with one `ssh` into the duckiebot, and type in above command in all three terminals. Start a **roscore** in one terminal by typing `$ roscore`.
### ROS Subscriber
The ROS script on the duckiebot contains a subscriber for motor command and a publisher for image acquisition. The motor command subscriber is `Duckiebot_Survey/catkin_ws/src/motor_control/src/motor_control.py` on the duckiebot side. This script is looks very similar to RR Drive Service because most part is the provided python class object. Inside the subscriber, there’s a function `listener()`, and this is the main part for ROS subscriber. 
```
rospy.init_node('motor_control', anonymous=True)
rospy.Subscriber("motor_command", Twist, callback)
```
Above lines initialize the ROS node name as *motor_control*, and subscribe to **ROS Topic** *motor_command*, with [geometry_msgs/Twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html) type of [ROS message](http://wiki.ros.org/msg). And import a ROS message type like Twist by 
```
from geometry_msgs.msg import Twist
```
The `callback()` function controls the motor based on messages received. And `rospy.spin()` makes the subscriber runs indefinitely, applies to everything above it in main function. To run this subscriber, simply type in 
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
You are provided with ROS subscriber `motor_control.py` for motor command and ROS publisher for image publishing, they should be running on the duckiebot side. Try to make the duckiebot do lane following in scripts `~/Duckiebot_Survey/catkin_ws/src/lane_following/src/lane_following.py` by filling in the `#TO DO` sections (search `TO DO` by `ctrl+F`). The main part is to complete the publisher for motor command and subsciber for image. Make sure to set **ROS_MASTER_URI** on both duckiebot and computer.

To test the scripts, open up three terminals and two `ssh` into the duckiebot. Go to `~/Duckiebot_Survey/catkin_ws` and run
```
$ source devel/setup.bash						#in all terminals
$ roslaunch raspicam_node camerav2_640x480.launch enable_raw:=true	#on duckiebot
$ python src/motor_control/src/motor_control.py				#on duckiebot
$ python src/lane_following/src/lane_following.py			#on computer

