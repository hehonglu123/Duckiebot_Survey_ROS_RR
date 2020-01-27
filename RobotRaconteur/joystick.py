import pygame

pygame.init()

# Used to manage how fast the pdates.
clock = pygame.time.Clock()

# Initialize the joysticks.
pygame.joystick.init()

# -------- Main Program Loop -----------
while True:
    #
    # EVENT PROCESSING STEP
    #
    # Possible joystick actions: JOYAXISMOTION, JOYBALLMOTION, JOYBUTTONDOWN,
    # JOYBUTTONUP, JOYHATMOTION
    for event in pygame.event.get(): # User did something.
        if event.type == pygame.JOYBUTTONDOWN:
            print("Joystick button pressed.")
        elif event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")


    # Get count of joysticks.
    joystick_count = pygame.joystick.get_count()


    # For each joystick:
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()

        if (joystick.get_axis(0)<-0.9):
            print('left')
        elif (joystick.get_axis(0)>0.9):
            print('right')
        if (joystick.get_axis(1)<-0.9):
            print('forward')
        elif (joystick.get_axis(1)>0.9):
            print('backward')


        if (joystick.get_button(0)==1):
            print('A')
        if (joystick.get_button(1)==1):
            print('B')
        if (joystick.get_button(2)==1):
            print('X')
        if (joystick.get_button(3)==1):
            print('Y')

    # Limit to 20 frames per second.
    clock.tick(20)

# Close the window and quit.
# If you forget this line, the program will 'hang'
# on exit if running from IDLE.
pygame.quit()