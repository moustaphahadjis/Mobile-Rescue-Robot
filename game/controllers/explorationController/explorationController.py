"""explorationController controller."""

from controller import Robot
robot = Robot()
timeStep = 32
max_velocity = 6.28

wheel_left = robot.getDevice("left wheel motor")
wheel_right = robot.getDevice("right wheel motor")
gyro = robot.getDevice("gyro")
gyro.enable(timeStep)

compass =robot.getDevice("compass")
compass.enable(timeStep)


# Declare colour sensor underneith the robot
colour_camera = robot.getDevice("colour_camera")
colour_camera.enable(timeStep)

# Declare communication link between the robot and the controller
emitter = robot.getDevice("emitter")

# Declare GPS
gps = robot.getDevice("gps")
gps.enable(timeStep)

# Declare heat/temperature sensor
left_heat_sensor = robot.getDevice("left_heat_sensor")
right_heat_sensor = robot.getDevice("right_heat_sensor")

left_heat_sensor.enable(timeStep)
right_heat_sensor.enable(timeStep)

rangeFinder = robot.getDevice('range-finder')
rangeFinder.enable(timeStep)
# Declare distance sensors around the robot
leftSensors = []
rightSensors = []
frontSensors = []

frontSensors.append(robot.getDevice("ps7"))
frontSensors[0].enable(timeStep)
frontSensors.append(robot.getDevice("ps0"))
frontSensors[1].enable(timeStep)

rightSensors.append(robot.getDevice("ps1"))
rightSensors[0].enable(timeStep)
rightSensors.append(robot.getDevice("ps2"))
rightSensors[1].enable(timeStep)

leftSensors.append(robot.getDevice("ps5"))
leftSensors[0].enable(timeStep)
leftSensors.append(robot.getDevice("ps6"))
leftSensors[1].enable(timeStep)

wheel_left.setPosition(float("inf"))
wheel_right.setPosition(float("inf"))
wheel_left.setPosition(0)
wheel_right.setPosition(0)
# Store when the program began
program_start = robot.getTime()

def move_backwards():
    speeds[0] = -0.5 * max_velocity
    speeds[1] = -0.7 * max_ve
def stop():
    speeds[0] = 0
    speeds[1] = 0

# Steer the robot right
def turn_right():
    wheel_right.setVelocity(0)
    wheel_left.setVelocity(0.1*max_velocity)

# Steer the robot left
def turn_left():
    wheel_right.setVelocity(0.1*max_velocity)
    wheel_left.setVelocity(0)

# Spin the robot on its spot
def spin():
    #set left wheel speed
    speeds[0] = 0.6 * max_velocity
    #set right wheel speed
    speeds[1] = -0.6 * max_velocity


def go_fornt():
    wheel_right.setVelocity(0.2 *max_velocity)
    wheel_left.setVelocity(0.2 *max_velocity)

def go_north():
    if(compass.getValues()[1]<= 0.9 and compass.getValues()[1]>= 1.1  and compass.getValues()[0]>= -0.1 and compass.getValues()[0]<= 0.1):
        go_fornt()

        print("go front")
    else:
        if compass.getValues()[0] > 0:
            turn_right()
        else: turn_left

camera = robot.getDevice("camera_left")
camera.enable(timeStep)
camera.recognitionEnable(timeStep)
rec = camera.getRecognitionNumberOfObjects()
while robot.step(timeStep) != -1:
    print(compass.getValues())
    go_north()
    pass