import math
from typing import Final
class Bot:
 def __init__(self,robot,wheel_right,wheel_left,gps,iu):
        self.robot = robot
        self.wheel_right = wheel_right
        self.wheel_left= wheel_left
        self.gps=gps
        self.iu = iu
        

        self.timeStep = 64
        self.max_velocity = 6.28
        self.dist_thr = 0.01
        self.dir_thr = 3
        
 def getLastPos(self):
     x: Final[float] = self.gps.getValues()[0]
     y: Final[float] = self.gps.getValues()[1]

     self.lastX,self.lastY = x,y
     
 def getYaw(self):
        v = self.iu.getRollPitchYaw()
        yaw = round(math.degrees(v[2]))
    # The InertialUnit gives us a reading based on how the robot is oriented with
    # respect to the X axis of the world: EAST 0°, NORTH 90°, WEST 180°, SOUTH -90°.
    # This operation converts these values to a normal, positive circumfrence.
        if yaw < 0:
            yaw += 360
        return yaw
 def stop(self):
    self.wheel_right.setVelocity(0)
    self.wheel_left.setVelocity(0)

# Steer the robot right
 def turn_right(self):
    self.wheel_right.setVelocity(-0.1*self.max_velocity)
    self.wheel_left.setVelocity(0.1*self.max_velocity)

# Steer the robot left
 def turn_left(self):
    self.wheel_right.setVelocity(0.1*self.max_velocity)
    self.wheel_left.setVelocity(-0.1*self.max_velocity)


 def traveled_distance(self):
    curr = self.gps.getValues()

    gps_x = self.lastX - curr[0]
    gps_y = self.lastY - curr[1]

    print(curr)
    print(self.lastX)
    print(self.lastY)
    val = math.sqrt(gps_x*gps_x + gps_y*gps_y)
    print(val)
    return val


 def go_front(self):
    if self.traveled_distance() < self.dist_thr:
        self.wheel_right.setVelocity(0.5 *self.max_velocity)
        self.wheel_left.setVelocity(0.5 *self.max_velocity)
    else:
        self.stop()

 def go_back(self):
    if self.traveled_distance() < self.dist_thr:
        self.wheel_right.setVelocity(-0.5 *self.max_velocity)
        self.wheel_left.setVelocity(-0.5 *self.max_velocity)

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

 def face_dir(self,dir):
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
    print(f'yaw is {self.getYaw()} and angle is {angle}')
    while self.robot.step(self.timeStep) != -1:
     if self.getYaw()< angle - self.dir_thr:
        self.turn_left()
     elif self.getYaw()>angle + self.dir_thr:
        self.turn_right()
     else :
        self.stop()
        print(f'Facing {dir}')
        resp = True
        break
    
    if resp:
        self.go_front()
    else:
        self.stop()