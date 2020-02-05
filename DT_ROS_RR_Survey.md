# Duckiebot ROS and RR Survey
## Introduction
In this survey, we want students to learn and compare robotics middleware: widely used [Robot Operating System](http://wiki.ros.org/) (ROS) and [Robot Raconteur](https://www.robotraconteur.com/) (RR) designed by RPI alumni. Given a duckiebot with [Ubuntu Mate (18.04)](https://ubuntu-mate.org/blog/ubuntu-mate-bionic-final-release/) image, the goal is to use ROS and Robot Raconteur to achieve joystick control with a little image processing.
On the computer/duckiebot, please follow instructions on [RR website](https://github.com/robotraconteur/robotraconteur/wiki/Download) (Ubuntu Xenial python2 version) and [ROS website](http://wiki.ros.org/melodic/Installation/Ubuntu) (Desktop version) to install both of them.
## Duckiebot
[Duckiebot](https://www.duckietown.org/) is a wheeled robot with 2 motors, one Raspberry Pi, a motor HAT and a Pi Camera. You have the control over 2 motor wheels and read image or video from Pi Cam. To access the duckiebot, use **ssh** command with given *username*, *password* and *IP address*:
```
ssh <username>@<IP>
```
First, clone this repository to the root directory on duckiebot and computer
```
cd ~
git clone https://github.com/hehonglu123/Duckiebot_Survey_ROS_RR.git
```

## Robot Raconteur Survey
Robot Raconteur is an object oriented Service-Client middleware. An RR service generally runs with a hardware (e.g. sensors,actuators) attached to a robot/computer to have direct communication between them. An RR client can receive messages that are sent from services and can call object functions in the service to command the robot. In this survey, we’ll first demonstrate how RR works with virtual duckiebot keyboard control. 
### RR Service:
First go to `Duckiebot_Survey_ROS_RR/RobotRaconteur/Example`, there is RR service script `service.py`. Simply run it by `python service.py`, and it will bring up a `pygame` window, with a virtual Duckiebot at upper left corner. Now open `service.py` in an Editor, and see how the RR service actually works. 
In order to use Robot Raconteur library as RR service, it’s necessary to import RR library at start:
```
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
```
Right after that, there's the object definition:
```
minimal_create_interface="""
#Service to provide virtual interface to the Duckiebot Drive
service experimental.minimal_create
stdver 0.9
object create_obj
     function void Drive(single x_vel, single y_vel)
end object
"""
```
In this example, there's only one object called `create_obj`, with a void function called `Drive` which takes in 2 single precision float variables, correspoinding to horizontal and vertical velocity. Now take a look at the python class `create_impl` below. Upon initialization, most part is pygame initialization and also virtual Duckiebot location initialization. The function `Drive` is the actual function that moves the virtual Duckiebot, and note the `Drive` function defined in RR object definition before refers to this function.
Inside main function, the RR service node is created
```
with RR.ServerNodeSetup("experimental.minimal_create", 52222):     
```
at port 52222 (you can choose any port as long as not used by other services). 
And the service type is registered by 
```
RRN.RegisterServiceType(minimal_create_interface)         
```
Now we create a python class object `create_inst=create_impl(0,0)`. The major difference for Robot Raconteur is that it has security over service. The password is hashed and a username is also required to connect to the service. 
```
authdata="cats be7af03a538bf30343a501cb1c8237a0 objectlock"`
```
Here the username is **cats** and password is **cats111!** after hashed. You can create your own username and password through MD5 hash (e.g. `$ echo -n cats111! | md5sum`). The final security policy is stored in `security=RR.ServiceSecurityPolicy(p,policies)`, and passed to service registration together with the object, 
```
RRN.RegisterService("Create","experimental.minimal_create.create_obj",create_inst,security)
```
with service name `Create`. The `security` argument is actually optional. Without that, any client on network can specify the service IP address to connect.
Even though `create_inst` is the python class object, the object received on client side only has attributes defined in RR object definition earlier.

### RR Client:
As for the RR client `client.py`, the first step is also to import RR client library:
```
from RobotRaconteur.Client import *
```
Most part of this script is pygame interfaces, so the RR client part is inside main function,
```
url='rr+tcp://localhost:52222/?service=Create'
username="cats"
password={"password":RR.RobotRaconteurVarValue("cats111!","string")}
obj=RRN.ConnectService(url,username,password)
```
The client connects to `Create` service at localhost port 52222. If the client is on another machine, simply swap localhost with IP address (or hostname) the servie is on. The username and password have to match with the ones on service side, otherwise you'll get authentification error. Note that username and password are optional, so if the service doesn't have them, just drop those two argument in `ConnectService`.
Then the `obj` object is passed to function `loop()`, which detects the keyboard arrow keys pressed or not. For example, if left arrow key is pressed, then 
```
obj.Drive(-0.5,0)
```
Try running this client on another terminal and play with it. Keep in mind the service always has to start before the client.

### Task 1: Motor Driving
Inside `~/Duckiebot_Survey_ROS_RR/RobotRaconteur/`, there's a scirpt called `Example_Drive.py`. This script can run directly, and makes the motor drive straight for 5 seconds. The motor drivers are located in the same directory, and the task is to fill in `#TODO` section to make this an RR motor drive service. After that, try create an RR client script on your laptop to drive the duckiebot motor remotely.

### Task 2: Joystick Command
You are provided with a joystick and `joystick.py` that reads in joystick command. The goal is to write an RR client that drives the motor accordingly based on the joystick command.

### Task 3: PiCam Streaming
Given examples for webcam service (`SimpleWebcamService.py`) and client (`SimpleWebcamClient_streaming.py`), write RR service and client for the Picam on duckiebot, so that on the computer side you can get video streaming from the Picam. Picam python package is already installed, and their API is listed here: https://picamera.readthedocs.io/en/release-1.10/recipes1.html. The service has to run on the duckiebot side, and the client may be on any device over the network.

### Task 4: Emergency "Stop"
Based on the Picam client from Task 1 and joystick motor control client from Task 3, the goal is to integrate them together with a little image processing. There will be "Stop" sign on the path, so when you are driving the duckiebot and there's a "Stop" sign too close, the duckiebot shall stop immediately. 
First modify the client from Task 1 to add "Stop" sign detection: taken the fact that the "Stop" sign is almost red, try filter out all red pixels ([cv2.inRange](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html)) and go through a connected component lableling ([cv2.connectedComponentsWithStats](https://docs.opencv.org/3.4/d3/dc0/group__imgproc__shape.html#ga107a78bf7cd25dec05fb4dfc5c9e765f)). Then integrate this with the client from Task 3: if the final number of pixels are larger than a threshold, the duckiebot shall stop. 

## ROS Survey
The structure of ROS is a little different from Robot Raconteur. First it has the Publisher-Subscriber relationship between different nodes. The example for ROS is turtlesim, a turtle vehicle simulator came with ROS installation. Go to `~/Duckiebot_Survey_ROS_RR/ROS/Example` first, then open up 4 terminals on the computer, with one typed in `$ roscore`, one typed in `$ rosrun turtlesim turtlesim_node`, one typed in `python move.py`, and the last one typed in `$ rostopic echo /turtle1/cmd_vel`. Give command in the terminal running `move.py`, and monitor the message in the `echo` terminal.
Each ROS communication requires one and only one `roscore` running. The command `rosrun turtlesim turtlesim_node` brings up the turtlesim environment as well as a ROS **subscriber** subscribing **rostopic** `/turtle1/cmd_vel`. The script `move.py` is known as the ROS **publisher** that publish the message to the same **rostopic**. 
Open up `move.py` and take a look at how it's done. First the ROS libraries and messages are imported:
```
import rospy
from geometry_msgs.msg import Twist
```
The [ROS message](http://wiki.ros.org/msg) are a standardized series of messages that used for communication between publisher and subscriber, and here to publish velocity data, we are using [gemoetry_msgs/Twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html). Inside `move()` function, the ROS node, publisher and message type are initialized:
```
rospy.init_node('robot_cleaner', anonymous=True)
velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) #Note the rostopic, message type are specified 
vel_msg = Twist()
```
After the script reads in user input data, the message is formed by setting values of each component (`linear` and `angular`). The program keeps running inside the `while` loop, and at each iteration, the message is published to **rostopic** `/turtle1/cmd_vel` by `velocity_publisher.publish(vel_msg)`. Take a look at `rotate.py`, which commands the turtle to rotate and get a sense how this can be combined to command the turtle to specified location.
Now in the terminal running `move.py`, terminate it by `ctrl+c` and run `$ python gotogoal.py`. Specify the location and tolerance and monitor the ros message in the `echo` terminal. 
Now open the script `gotogoal.py` in editor; similarly, the ROS library and messages are imported at the top:
```
import rospy
from geometry_msgs.msg  import Twist
from turtlesim.msg import Pose
```
, and the initialization looks similar except there's a subscriber now:
```
self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
```
This subscriber subscribes to the **rostopic** `/turtle1/pose` with [`Pose`](https://github.com/ros/ros_tutorials/blob/melodic-devel/turtlesim/msg/Pose.msg) type of message. Everytime a message is subscribed, the `callback()` function will be triggered, so that this script can always get the current location of the turtle to feed into the feedback loop. Depending on current location and desired location specified by the user, the linear and angular velocity are calculated out and published to **rostopic** `/turtle1/cmd_vel` by `self.velocity_publisher.publish(vel_msg)`. At last, `rospy.spin()` keeps this script runs indefinitely until interruption.

### Task 1: Motor Driving
The first task will ask you to write a subscriber that subscribes to a **rostopic** with `Twist` type of message and drive the motor accordingly (set linear.x for left motor and linear.y for right motor). First open three terminals `ssh` into the duckiebot, with one terminal running `roscore`. Inside `~/Duckiebot_Survey_ROS_RR/ROS/`, there's a scirpt called `Example_Drive.py`. This script can run directly, and makes the motor drive straight for 5 seconds. The motor drivers are located in the same directory, and the task is to fill in `#TODO` section to make this a ROS subscriber. After that, run the script directly, and in the other terminal, run `$ rostopic pub <topic-name> <topic-type> [data...]` (replace `<topic-name>` with your **rostopic** name and keep pressing `TAB`, then modify `linear.x` and `linear.y`). You can always monitor the message by `rostopic echo <topic-name>`.

Once the duckiebot responds to the publisher, let's try doing this remotely by setting **ROS MASTER**. Stop all scripts including `roscore` on Duckiebot. 
To initiate a ROS communication from computer to the Duckiebot, it’s necessary to identify which one is **ROS MASTER**. This needs to be done on both side by 
`$ export ROS_MASTER_URI=http://<hostname>:11311`
, where the hostname is the laptop’s hostname or IP address. Once this is done, you can look up this value by 
`$ echo $ROS_MASTER_URI`
to make sure it’s set. Note you need to do this for every terminal opened, and you only need one roscore running in one Master-Slave setup, which should be on the laptop side. This only needs to be done if you need ROS communication among different machines.
So for teleop, we have communication between Duckiebot and computer. On the computer side, make sure all terminals are set with **ROS_MASTER**, and with one running `roscore`. Then try the `$ rostopic pub` command on the computer side, and check if the duckiebot receives the messages or not. 

### Task 2: Joystick Command
You are provided with a joystick and `joystick.py` that reads in joystick command. The goal is to write an ROS publisher that teleop Duckiebot. Try run `joystick.py` directly and see how it works, then add ROS part into it. Keep in mind that one and only one `roscore` should be running, and remember to set **ROS MASTER** in every terminal.

### Task 3: PiCam Streaming
Given examples for webcam publisher and subscriber (`Example_Webcam_Pub.py` and `Example_Webcam_Subscriber.py`), write another ROS publisher and subscriber for the Picam on Duckiebot, so that on the computer side you can get video streaming from the Picam. Picam python package is already installed, and their API is listed here: https://picamera.readthedocs.io/en/release-1.13/api_streams.html. The publisher has to run on the duckiebot side, and the subscriber may be on any device over the network.


### Task 4: Emergency "Stop"
Based on the Picam publisher from Task 3 and script for joystick motor control from Task 2, the goal is to integrate them together with a little image processing. There will be "Stop" sign on the path, so when you are driving the duckiebot and there's a "Stop" sign too close, the duckiebot shall stop immediately. 
First modify the subscriber from Task 3 to add "Stop" sign detection: taken the fact that the "Stop" sign is almost red, try filter out all red pixels ([cv2.inRange](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html)) and go through a connected component lableling ([cv2.connectedComponentsWithStats](https://docs.opencv.org/3.4/d3/dc0/group__imgproc__shape.html#ga107a78bf7cd25dec05fb4dfc5c9e765f)). Then integrate this with the publisher from Task 2: if the final number of pixels are larger than a threshold, the Duckiebot shall stop. 
