import os
import pygame
from pygame.math import Vector2
import time

#import RR library
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s

#object definition
minimal_create_interface="""
#Service to provide virtual interface to the Duckiebot Drive
service experimental.minimal_create

stdver 0.9

object create_obj
     function void Drive(single x_vel, single y_vel)

end object
"""
#Actual class object
class create_impl:
    def __init__(self, x, y):               #initialization upon creation, mostly pygame setup
        self.position = Vector2(x, y)
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
        velocity=Vector2(x_vel,y_vel)

        self.position += velocity
        self.screen.fill((0, 0, 0))
        self.rect = self.car_image.get_rect()
        self.screen.blit(self.car_image, self.position * 32 - (self.rect.width / 2, self.rect.height / 2))
        pygame.display.flip()


if __name__ == '__main__':
        
    with RR.ServerNodeSetup("experimental.minimal_create", 52222):      #setup RR node with service name and port
        #Register the service type
        RRN.RegisterServiceType(minimal_create_interface)               #register service type

        create_inst=create_impl(0,0)                #create object                      

        #Register the service with definition and object
        RRN.RegisterService("Create","experimental.minimal_create.create_obj",create_inst)

        #Wait for program exit to quit
        raw_input("Press enter to quit")
        pygame.quit()

   