import math
import time
import random
from controller import Robot

# Definition of devices
BUMPERS_NUMBER = 2
BUMPER_LEFT = 0
BUMPER_RIGHT = 1
bumpers = [None] * BUMPERS_NUMBER
bumpers_name = ["bumper_left", "bumper_right"]

CLIFF_SENSORS_NUMBER = 4
CLIFF_SENSOR_LEFT = 0
CLIFF_SENSOR_FRONT_LEFT = 1
CLIFF_SENSOR_FRONT_RIGHT = 2
CLIFF_SENSOR_RIGHT = 3
cliff_sensors = [None] * CLIFF_SENSORS_NUMBER
cliff_sensors_name = ["cliff_left", "cliff_front_left", "cliff_front_right", "cliff_right"]

LEDS_NUMBER = 3
LED_ON = 0
LED_PLAY = 1
LED_STEP = 2
leds = [None] * LEDS_NUMBER
leds_name = ["led_on", "led_play", "led_step"]

receiver = None
receiver_name = "receiver"

left_motor, right_motor, left_position_sensor, right_position_sensor = None, None, None, None

# Misc Stuff
MAX_SPEED = 16
NULL_SPEED = 0
HALF_SPEED = 8
MIN_SPEED = -16

WHEEL_RADIUS = 0.031
AXLE_LENGTH = 0.271756
ENCODER_RESOLUTION = 507.9188

# Helper functions
def get_time_step():
    return int(robot.getBasicTimeStep())

def step():
    if robot.step(get_time_step()) == -1:
        robot.cleanup()
        import sys
        sys.exit()

def init_devices():
    global receiver, left_motor, right_motor, left_position_sensor, right_position_sensor
    for i in range(LEDS_NUMBER):
        leds[i] = robot.getDevice(leds_name[i])

    for i in range(BUMPERS_NUMBER):
        bumpers[i] = robot.getDevice(bumpers_name[i])
        bumpers[i].enable(get_time_step())

    for i in range(CLIFF_SENSORS_NUMBER):
        cliff_sensors[i] = robot.getDevice(cliff_sensors_name[i])
        cliff_sensors[i].enable(get_time_step())

    receiver = robot.getDevice(receiver_name)
    receiver.enable(get_time_step())

    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    left_position_sensor = robot.getDevice("left wheel sensor")
    right_position_sensor = robot.getDevice("right wheel sensor")
    left_position_sensor.enable(get_time_step())
    right_position_sensor.enable(get_time_step())

def is_there_a_collision_at_left():
    return bumpers[BUMPER_LEFT].getValue() != 0.0

def is_there_a_collision_at_right():
    return bumpers[BUMPER_RIGHT].getValue() != 0.0

def fflush_ir_receiver():
    while receiver.getQueueLength() > 0:
        receiver.nextPacket()

def is_there_a_virtual_wall():
    return receiver.getQueueLength() > 0

def is_there_a_cliff_at_left():
    return cliff_sensors[CLIFF_SENSOR_LEFT].getValue() < 100.0 or cliff_sensors[CLIFF_SENSOR_FRONT_LEFT].getValue() < 100.0

def is_there_a_cliff_at_right():
    return cliff_sensors[CLIFF_SENSOR_RIGHT].getValue() < 100.0 or cliff_sensors[CLIFF_SENSOR_FRONT_RIGHT].getValue() < 100.0

def is_there_a_cliff_at_front():
    return cliff_sensors[CLIFF_SENSOR_FRONT_LEFT].getValue() < 100.0 or cliff_sensors[CLIFF_SENSOR_FRONT_RIGHT].getValue() < 100.0

def go_forward():
    left_motor.setVelocity(MAX_SPEED)
    right_motor.setVelocity(MAX_SPEED)

def go_backward():
    left_motor.setVelocity(-HALF_SPEED)
    right_motor.setVelocity(-HALF_SPEED)

def stop():
    left_motor.setVelocity(-NULL_SPEED)
    right_motor.setVelocity(-NULL_SPEED)

def passive_wait(sec):
    start_time = robot.getTime()
    while start_time + sec > robot.getTime():
        step()

def randdouble():
    return random.random()

def turn(angle):
    stop()
    l_offset = left_position_sensor.getValue()
    r_offset = right_position_sensor.getValue()
    step()
    neg = -1.0 if angle < 0.0 else 1.0
    left_motor.setVelocity(neg * HALF_SPEED)
    right_motor.setVelocity(-neg * HALF_SPEED)
    orientation = 0.0
    while orientation < neg * angle:
        l = left_position_sensor.getValue() - l_offset
        r = right_position_sensor.getValue() - r_offset
        dl = l * WHEEL_RADIUS  # distance covered by left wheel in meter
        dr = r * WHEEL_RADIUS  # distance covered by right wheel in meter
        orientation = neg * (dl - dr) / AXLE_LENGTH  # delta orientation in radian
        step()
    stop()
    step()

# main
robot = Robot()
print("Default controller of the iRobot Create robot started...")

init_devices()

leds[LED_ON].set(True)
passive_wait(0.5)

while True:
    if is_there_a_virtual_wall():
        print("Virtual wall detected")
        turn(math.pi)
    elif is_there_a_collision_at_left() or is_there_a_cliff_at_left():
        print("Left obstacle detected")
        go_backward()
        passive_wait(0.5)
        turn(math.pi * randdouble())
    elif is_there_a_collision_at_right() or is_there_a_cliff_at_right() or is_there_a_cliff_at_front():
        print("Right obstacle detected")
        go_backward()
        passive_wait(0.5)
        turn(-math.pi * randdouble())
    else:
        go_forward()
    fflush_ir_receiver()
    step()
