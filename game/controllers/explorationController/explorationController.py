"""explorationController controller."""
import math
from world import World
from matplotlib import pyplot as plt
from controller import Robot

robot = Robot()
timeStep =64

facing = None

wheel_left = robot.getDevice("left wheel motor")
wheel_right = robot.getDevice("right wheel motor")
gyro = robot.getDevice("gyro")
gyro.enable(timeStep)

compass =robot.getDevice("compass")
compass.enable(timeStep)

iu = robot.getDevice('inertial unit')
iu.enable(timeStep)

def getYaw():
    v = iu.getRollPitchYaw()
    yaw = round(math.degrees(v[2]))
    # The InertialUnit gives us a reading based on how the robot is oriented with
    # respect to the X axis of the world: EAST 0°, NORTH 90°, WEST 180°, SOUTH -90°.
    # This operation converts these values to a normal, positive circumfrence.
    if yaw < 0:
        yaw += 360
    return yaw

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
wheel_left.setVelocity(0)
wheel_right.setVelocity(0)
# Store when the program began
program_start = robot.getTime()


def stop():
    wheel_right.setVelocity(0)
    wheel_left.setVelocity(0)

# Steer the robot right
def turn_right():
    wheel_right.setVelocity(-0.1*max_velocity)
    wheel_left.setVelocity(0.1*max_velocity)

# Steer the robot left
def turn_left():
    wheel_right.setVelocity(0.1*max_velocity)
    wheel_left.setVelocity(-0.1*max_velocity)


def traveled_distance():
    curr = gps.getValues()

    gps_x = lastPos[0] - curr[0]
    gps_y = lastPos[1] - curr[1]

    print(curr)
    print(lastPos)
    val = math.sqrt(gps_x*gps_x + gps_y*gps_y)
    print(val)
    return val

dist_thr = 0.01
def go_front():
    if traveled_distance() < dist_thr:
        wheel_right.setVelocity(0.5 *max_velocity)
        wheel_left.setVelocity(0.5 *max_velocity)
    else:
        stop()

def go_back():
    if traveled_distance() < dist_thr:
        wheel_right.setVelocity(-0.5 *max_velocity)
        wheel_left.setVelocity(-0.5 *max_velocity)

def find_highest_lowest(numbers):
    if not numbers:
        return None, None  # Return None for both highest and lowest if the list is empty

    highest = lowest = numbers[0]  # Initialize highest and lowest with the first element of the list

    for num in numbers[1:]:
        if num > highest:
            highest = num
        elif num < lowest:
            lowest = num

    return highest, lowest

dir_thr = 3
def face_dir(dir):
    # north south east west

    resp = False
    angle = 0
    if dir == 'north':
        angle = 272
    elif dir == 'east':
        angle = 180
    elif dir == 'west':
        angle = 0
    elif dir == 'south':
        angle = 90
    else:
        print('direction input error')
        return False
    print(f'yaw is {getYaw()} and angle is {angle}')
    while robot.step(timeStep) != -1:
     if getYaw()< angle - dir_thr:
        turn_left()
     elif getYaw()>angle + dir_thr:
        turn_right()
     else :
        stop()
        print(f'Facing {dir}')
        resp = True
        break
    
    if resp:
        go_front()
    else:
        stop()


def go_north(lastPosition):
    resp = False
    if getYaw()< 272 - dir_thr:
        turn_left()
    elif getYaw()>272 + dir_thr:
        turn_right()
    else :
        if(lastPosition[0] < gps.getValues()[0]):
            go_front()
        else:
            print('stop')
            stop()
            resp = True
    
    return resp


laser = robot.getDevice("laser")
laser.enable(timeStep)
camera = robot.getDevice("camera_left")
camera.enable(timeStep)
camera.recognitionEnable(timeStep)
rec = camera.getRecognitionNumberOfObjects()




robot.step(1000)

world =World(100,100)
from bot import Bot
def main():
    
    bot = Bot(robot,wheel_right,wheel_left,gps,iu)
    bot.getLastPos()
    while robot.step() != -1:
        map = World(100,100)
        bot.face_dir('north')
            #print(laser.getValue())
        continue
        #plt.plot([1],[1])


if __name__ == '__main__':
    main()

