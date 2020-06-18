from RobotRaconteur.Client import *		#import RR client library
import time
import pygame
import os, sys
import re

#pygame initialization
screen_size = 300
speed_tang = 1.0
speed_norm = 1.0


time_to_wait = 10000
last_ms = 0

last_ms_p = 0

auto_restart = False

def loop(obj):
    global last_ms, time_to_wait, last_ms_p
    veh_standing = True

    while True:

        # add dpad to screen
        screen.blit(dpad, (0,0))

        # obtain pressed keys
        keys = pygame.key.get_pressed()

        ### checking keys and executing actions ###

        # drive left
        if keys[pygame.K_LEFT]:
            obj.Drive(-0.5,0)			####Drive left
            screen.blit(dpad_l, (0,0))

        # drive right
        if keys[pygame.K_RIGHT]:
            obj.Drive(0.5,0)			####Drive right
            screen.blit(dpad_r, (0,0))

        # drive forward
        if keys[pygame.K_UP]:
            obj.Drive(0,-0.5)			####Drive up
            screen.blit(dpad_f, (0,0))

        # drive backwards
        if keys[pygame.K_DOWN]:
            obj.Drive(0,0.5)			####Drive down
            screen.blit(dpad_b, (0,0))



        ## key/action for quitting the program

        # check if top left [x] was hit
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()

        # quit program
        if keys[pygame.K_q]:
            pygame.quit()

        ### END CHECKING KEYS ###

        # refresh screen
        pygame.display.flip()

        # check for any input commands
    

        # adjust veh_standing such that when vehicle stands still, at least
        # one last publishment was sent to the bot. That's why this adjustment
        # is made after the publishment of the message


        time.sleep(0.03)

        # obtain next key list
        pygame.event.pump()


# prepare size and rotations of dpad and dpad_pressed
def prepare_dpad():
    global dpad, dpad_f, dpad_r, dpad_b, dpad_l
    file_dir = os.path.dirname(__file__)
    file_dir = (file_dir + "/") if  (file_dir) else ""

    dpad = pygame.image.load(file_dir + "images/d-pad.png")
    dpad = pygame.transform.scale(dpad, (screen_size, screen_size))
    dpad_pressed = pygame.image.load(file_dir + "images/d-pad-pressed.png")
    dpad_pressed = pygame.transform.scale(dpad_pressed, (screen_size, screen_size))
    dpad_f = dpad_pressed
    dpad_r = pygame.transform.rotate(dpad_pressed, 270)
    dpad_b = pygame.transform.rotate(dpad_pressed, 180)
    dpad_l = pygame.transform.rotate(dpad_pressed, 90)

# Hint which is print at startup in console
def print_hint():
    print("\n\n\n")
    print("Virtual Joystick for your Duckiebot")
    print("-----------------------------------")
    print("\n")
    print("[ARROW_KEYS]:    Use them to steer your Duckiebot")
    print("         [q]:    Quit the program")
    print("         [a]:    Start lane-following a.k.a. autopilot")
    print("         [s]:    Stop lane-following")
    print("         [i]:    Toggle anti-instagram")
    print("\n")
    print("Questions? Contact Julien Kindle: jkindle@ethz.ch")



if __name__ == '__main__':

	# prepare pygame
    pygame.init()

    file_dir = os.path.dirname(__file__)
    file_dir = (file_dir + "/") if  (file_dir) else ""
    logo = pygame.image.load(file_dir + "images/logo.png")

    pygame.display.set_icon(logo)
    screen = pygame.display.set_mode((screen_size,screen_size))


    prepare_dpad()
    print_hint()

    #Connect to RR service
    url='rr+tcp://localhost:52222/?service=Create'
    obj=RRN.ConnectService(url)
    loop(obj)
