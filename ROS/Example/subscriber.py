import os
import pygame
import time
import numpy as np

#import ROS library
import rospy
from geometry_msgs.msg import Twist
#Actual class object
class create_impl:
    def __init__(self, x, y):               #initialization upon creation, mostly pygame setup
        self.position = np.array([x, y])
        pygame.init()
        pygame.display.set_caption("Car tutorial")
        width = 1280
        height = 720
        self.screen = pygame.display.set_mode((width, height))
        clock = pygame.time.Clock()
        exit = False
        current_dir = os.path.dirname(os.path.abspath(__file__))
        image_path = os.path.join(current_dir, "duck.jpeg")
        self.car_image = pygame.image.load(image_path)
        self.screen.fill((0, 0, 0))
        self.rect = self.car_image.get_rect()
        self.screen.blit(self.car_image, self.position * 32 - (self.rect.width / 2, self.rect.height / 2))
        pygame.display.flip()

    def Drive(self,x_vel,y_vel):            #Drive function, update new position, this is the one referred in definition
        velocity=np.array([x_vel, y_vel])

        self.position += velocity
        self.screen.fill((0, 0, 0))
        self.rect = self.car_image.get_rect()
        self.screen.blit(self.car_image, self.position * 32. - np.array([self.rect.width / 2., self.rect.height / 2.]))
        pygame.display.flip()

obj= create_impl(0.,0.)
def callback(data):
    rospy.loginfo(data)
    obj.Drive(data.linear.x,data.linear.y)
def listener():
    
    rospy.init_node('motor_command_pub', anonymous=False)    
    rospy.Subscriber("motor_command", Twist, callback, queue_size = 1, buff_size=2**24)
    rospy.spin()

if __name__ == '__main__':
    print("running")
    listener()
   