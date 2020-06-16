#Simple example Robot Raconteur webcam service

#Note: This example is intended to demonstrate Robot Raconteur
#and is designed to be simple rather than optimal.

import time
import RobotRaconteur as RR
#Convenience shorthand to the default node.
#RRN is equivalent to RR.RobotRaconteurNode.s
RRN=RR.RobotRaconteurNode.s
import threading
import numpy
import traceback
import cv2
import platform
import sys
import argparse

#Class that implements a single webcam
class Webcam_impl(object):
    #Init the camera being passed the camera number and the camera name
    def __init__(self,cameraid,cameraname):
        self._lock=threading.RLock()
        self._framestream=None
        self._framestream_endpoints=dict()
        self._framestream_endpoints_lock=threading.RLock()
        self._streaming=False
        self._cameraname=cameraname

        #Create buffers for memory members
        self._buffer=numpy.array([],dtype="u1")
        self._multidimbuffer=numpy.array([],dtype="u1")

        #Initialize the camera
        with self._lock:
            if platform.system() == "Windows":
                self._capture=cv2.VideoCapture(cameraid + cv2.CAP_DSHOW)
            else:
                self._capture=cv2.VideoCapture(cameraid)
            self._capture.set(3,320)
            self._capture.set(4,240)

    #Return the camera name
    @property
    def Name(self):
        return self._cameraname

    #Capture a frame and return a WebcamImage structure to the client
    def CaptureFrame(self):
        with self._lock:
            image=RRN.NewStructure("experimental.createwebcam2.WebcamImage")
            ret, frame=self._capture.read()
            if not ret:
                raise Exception("Could not read from webcam")
            image.width=frame.shape[1]
            image.height=frame.shape[0]
            image.step=frame.shape[1]*3
            image.data=frame.reshape(frame.size, order='C')

            return image

    #Shutdown the Webcam
    def Shutdown(self):
        self._streaming=False
        del(self._capture)


#A root class that provides access to multiple cameras
class WebcamHost_impl(object):
    def __init__(self,camera_names):
        cams=dict()
        for i in camera_names:
            ind,name=i
            cam=Webcam_impl(ind,name)
            cams[ind]=cam

        self._cams=cams


    #Returns a map (dict in Python) of the camera names
    @property
    def WebcamNames(self):
        o=dict()
        for ind in self._cams.keys():
            name=self._cams[ind].Name
            o[ind]=name
        return o

    #objref function to return Webcam objects
    def get_Webcams(self,ind):
        #The index for the object may come as a string, so convert to int
        #before using. This is only necessary in Python
        int_ind=int(ind)

        #Return the object and the Robot Raconteur type of the object
        return self._cams[int_ind], "experimental.createwebcam2.Webcam"

    #Shutdown all the webcams
    def Shutdown(self):
        for cam in self._cams.values():
            cam.Shutdown()



def main():

    with RR.ServerNodeSetup("Webcam_Service",2355) as node_setup:

        #Initialize the webcam host root object
        camera_names=[(0,"Cam1")]
        obj=WebcamHost_impl(camera_names)
        
        RRN.RegisterServiceTypeFromFile("experimental.createwebcam2")
        RRN.RegisterService("Webcam","experimental.createwebcam2.WebcamHost",obj)
    
        c1=obj.get_Webcams(0)[0]

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("Server started, press enter to quit...")
        else:
            raw_input("Server started, press enter to quit...")

        #Shutdown
        obj.Shutdown()    


if __name__ == '__main__':
    main()
