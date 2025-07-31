import lift_controller
import arx_lift_python as arx
import time
import pygame

control_loop = arx.LiftHeadControlLoop("can5",arx.RobotType.X7S)
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)

running_state = 2
lift_height = 0.0
while(1):
    pygame.event.get()
    lift_height += -joystick.get_axis(1) * 0.002
    if (joystick.get_button(0) == 1):
        running_state = 1
    if (joystick.get_button(1) == 1):
        running_state = 2
    print("head_yaw:"+"%f"%control_loop.get_head_yaw())
    print("head_pitch:"+"%f"%control_loop.get_head_pitch())
    print("height:"+"%f"%control_loop.get_height())
    control_loop.get_height()
    control_loop.set_head_yaw(0.0)
    control_loop.set_head_pitch(0.0)
    control_loop.set_waist_pos(0.0)
    control_loop.set_height(0.0)
    control_loop.set_chassis_cmd(joystick.get_axis(4) * -2,joystick.get_axis(3) * -2,joystick.get_axis(0) * -4,running_state)
    control_loop.loop()
    time.sleep(0.002)
