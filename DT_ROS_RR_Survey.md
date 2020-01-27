# Duckiebot ROS and RR Survey
## Introduction
In this survey, we want students to learn and compare robotics middleware: widely used [Robot Operating System](http://wiki.ros.org/) (ROS) and [Robot Raconteur](https://www.robotraconteur.com/) (RR) designed by RPI alumni. Given a duckiebot with [Ubuntu Mate (18.04)](https://ubuntu-mate.org/blog/ubuntu-mate-bionic-final-release/) image, the goal is to use ROS and Robot Raconteur to achieve lane following.
On the computer/duckiebot, please follow instructions on [RR website](https://github.com/robotraconteur/robotraconteur/wiki/Download) (Ubuntu Xenial python2 version) and [ROS website](http://wiki.ros.org/melodic/Installation/Ubuntu) (Desktop version) to install both of them.
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
### Task 1: PiCam Streaming
Given above examples for webcam service and client, write RR service and client for the Picam on duckiebot, so that on the computer side you can get video streaming from the Picam. Picam python package is already installed, and their API is listed here: https://picamera.readthedocs.io/en/release-1.13/api_streams.html. The service has to run on the duckiebot side, and the client may be on any device over the network.


### Task 2: Motor Driving
Inside `~/Duckiebot_Survey_ROS_RR/RobotRaconteur/`, there's a scirpt called `Example_Drive.py`. This script can run directly, and makes the motor drive straight for 5 seconds. The motor drivers are located in the same directory, and the task is to fill in `#TODO` section to make this an RR motor drive service. After that, try create an RR client script on your laptop to drive the duckiebot motor remotely.

### Task 3: Joystick Command
You are provided with a joystick, the goal is to write an RR service that sends the command from joystick to the network. Try to conduct a simple client to check if you can get the value from the service, and then drive the motor accordingly based on the joystick command.

### Task 4: Emergency "Stop"
Based on the Picam client from Task 1 and joystick motor control client from Task 3, the goal is to integrate them together with a little image processing. There will be "Stop" sign on the path, so when you are driving the duckiebot and there's a "Stop" sign too close, the duckiebot shall stop immediately. 
First modify the client from Task 1 to add "Stop" sign detection: taken the fact that the "Stop" sign is almost red, try filter out all red pixels and go through a connected component lableling. Then integrate this with the client from Task 3: if the final number of pixels are larger than a threshold, the duckiebot shall stop. 

## ROS Survey
The structure of ROS is a little different from Robot Raconteur. First it has the Publisher-Subscriber relationship between different nodes. In our case the subscriber is on the duckiebot, listening to the speed command messages from remote Ubuntu laptop. And obviously the Ubuntu laptop is the publisher, so that user can publish command toward the duckiebot. Another relationship in ROS is Master-Slave. In order to use ROS in python, it’s necessary to `import rospy` at the start of each script.

### ROS Master
To initiate a ROS communication from laptop to the duckiebot, it’s necessary to identify which one is ROS_MASTER. This needs to be done on both side by 
`$ export ROS_MASTER_URI=http://<hostname>:11311`
, where the hostname is the laptop’s hostname or IP address. Once this is done, you can look up this value by 
`$ echo $ROS_MASTER_URI`
to make sure it’s set. Note you need to do this for every new terminal opened, and you only need one roscore running in one Master-Slave setup, which should be on the laptop side. This only needs to be done if you need ROS communication among different machines.
So for the keyboard control example, we have communication between duckiebot and laptop. On the laptop side, open up four terminals, with one `ssh` into the duckiebot, and type in above command in all four terminals. Start a **roscore** in one terminal (Master side) by typing `$ roscore`.

### ROS Publisher
The example for ROS is also the Webcam streaming. The file `Example_Webcam_Pub.py` inside `Duckiebot_Survey_ROS_RR/ROS/` is a simple webcam ROS python publisher. First the ROS libraries and messages are imported:
```
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
```
The [ROS sensor message](http://wiki.ros.org/sensor_msgs) are a standardized series of messages that used for communication between publisher and subscriber, and here to publish image data, we are using [sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html).
Then take a look at the `main()` function, the ROS node is initialized:
```
rospy.init_node('camera_node',anonymous=False)
camera_node = CameraNode()
```
A `CameraNode()` object is created, and `rospy.spin()` keeps the program running. Now let's dig into the `CameraNode()` object. The camera parameters and publisher are set when constructed.
```
self.pub_img= rospy.Publisher("image_raw",Image,queue_size=1)
```
This line specifies the **ROS Topic** to be published to is called *image_raw*, and the queue_size is the buffer size for publishing.
The main part is the function ` def grabAndPublish(self,publisher)`, where the image is captured through `rval,img_data = self.camera.read()`, and then converted from OpenCV image to ROS image type below. Then the data is published from this node by 
```
publisher.publish(image_msg)
```
Try running this publisher simply by 
```
python Example_Webcam_Pub.py
```
On the other terminal, type in  `rostopic list`, and you should see *image_raw* listed below. Then try `rostopic echo /image_raw`, this will display the message on this topic in the terminal. You can always check what topic is available by this technique. To stop a running ROS script, simply press `ctrl+c`. 

### ROS Subscriber
The ROS subscriber script is called `Example_Webcam_Subscriber.py` inside `Duckiebot_Survey_ROS_RR/ROS`. It's also necessary to import the ROS library and message first:
```
import rospy
from sensor_msgs.msg import Image
```
In the `main()` function, similar to the publisher part, it's the same to initialize the ros node first, and then initialize the subscriber:
```
rospy.init_node('stream_node', anonymous=True)
sub = rospy.Subscriber("/image_raw",Image,callback)
```
The `callback` is a callback function, defined above as `def callback(data)`. Every time a message is received from the topic *image_raw*,  the callback function is triggered. Inside the callback function, the data is first deserialized by `imgmsg_to_cv2` and then displayed in the window.

At the bottom part, `rospy.spin()` makes the subscriber runs indefinitely, applies to everything above it in main function. To run this subscriber, simply type in 
```
$ python Example_Webcam_Subscriber.py
```
And it's able to display the webcam image in real time.

### Task 1: PiCam Streaming
Given above examples for webcam publisher and subscriber, write another ROS publisher and subscriber for the Picam on duckiebot, so that on the computer side you can get video streaming from the Picam. Picam python package is already installed, and their API is listed here: https://picamera.readthedocs.io/en/release-1.13/api_streams.html. The publisher has to run on the duckiebot side, and the subscriber may be on any device over the network.

### Task 2: Motor Driving
Inside `~/Duckiebot_Survey_ROS_RR/ROS/`, there's a scirpt called `Example_Drive.py`. This script can run directly, and makes the motor drive straight for 5 seconds. The motor drivers are located in the same directory, and the task is to fill in `#TODO` section to make this an ROS subscriber. After that, try create an ROS publisher script on your laptop to drive the duckiebot motor remotely.

### Task 3: Joystick Command
You are provided with a joystick, the goal is to write an ROS publisher that sends the command from joystick to the network. Try to conduct a simple subsciber to check if you can get the command from the publisher, and then combine with the publisher in Task 2 to drive the motor accordingly based on the joystick command.

### Task 4: Emergency "Stop"
Based on the Picam publisher from Task 1 and script for joystick motor control from Task 3, the goal is to integrate them together with a little image processing. There will be "Stop" sign on the path, so when you are driving the duckiebot and there's a "Stop" sign too close, the duckiebot shall stop immediately. 
First modify the client from Task 1 to add "Stop" sign detection: taken the fact that the "Stop" sign is almost red, try filter out all red pixels and go through a connected component lableling. Then integrate this with the client from Task 3: if the final number of pixels are larger than a threshold, the duckiebot shall stop. 
