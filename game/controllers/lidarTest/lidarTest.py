"""lidarTest controller."""
import math
import numpy as np
from move import Move
from map import Map
from detection import Detection

from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())


compass = robot.getDevice('compass')
compass.enable(timestep)

display = robot.getDevice('display')
displayRes = 1000
scale = 0.5
pos = [displayRes/2, displayRes/2]

gps = robot.getDevice('gps')
gps.enable(timestep)
initGPS = gps.getValues()

leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")

laser1 = robot.getDevice('laser1')
laser2 = robot.getDevice('laser2')
laser3 = robot.getDevice('laser3')
laser1.enable(timestep)
laser2.enable(timestep)
laser3.enable(timestep)
def getLaser(dist,currentPos):
    angle = math.radians(180)
    y = dist* math.sin(angle) 
    x = dist* math.cos(angle) 
    print(f'[{x, y}]')
    return [ x*displayRes*scale + currentPos[0] , -y*displayRes*scale + currentPos[1] ]
#Get angle of each Beam
#totalAngle = 6.283
totalAngle = 180
numBeams = 3
anglePerBeam = totalAngle/(numBeams-1)

def getBeams(robotOrientation, values, currentPos):
    result = np.zeros((numBeams,3))
    #i = numBeams-1
    #while i >=0:
    for i in range(numBeams):
     if i == 0:
        angle = math.radians(i*anglePerBeam )  # + robotOrientation
        
        #print(angle)
        y = values[i] * math.sin(angle) 
        x = values[i] * math.cos(angle) 

        
        result[i][0] = x*displayRes*scale + currentPos[0] if (values[i]<0.5 and values[i]>0.05) else 0
        result[i][1] = y*displayRes*scale + currentPos[1] if (values[i]<0.5 and values[i]>0.05) else 0
        result[i][2] = (values[i])
        
    
    return result

def drawBeams(points,map):
    count = 0
    for i in range(numBeams):
        if(points[i][0] !=0 or points[i][1]!=0):
            #display.drawPixel(points[i][0],points[i][1])
            map.setWall(points[i][0],points[i][1])
            count+=1
    #print(count)

def drawTrack(initGPS, map):
    x0 = -gps.getValues()[0] + initGPS[0]
    y0 = -gps.getValues()[2] + initGPS[2]

    x = x0*displayRes*scale + pos[0]
    y = y0*displayRes*scale + pos[1]
    map.setTrack(x,y)
    return (x,y)
    #print(f'{x,y}')

def getOrientation():
    rad = math.atan2(compass.getValues()[1], compass.getValues()[0])
    bearing = (rad - 1.5708) / 3.14159 * 180.0
    if (bearing < 0.0):
        bearing = bearing + 360.0
    return bearing


def main():
 map = Map(robot,timestep)
 move = Move(robot,timestep, map.lasers)
 detect = Detection(robot, timestep)

 robot.step(100)
 initGPS = gps.getValues()
 currentPos = (0,0)
 while robot.step(timestep) != -1:
    #vals = lidar.getRangeImage()
    #print(  lidar.getFov())
    #beams = getBeams(getOrientation(),vals, currentPos)
    #drawBeams(beams,map)
        #currentPos = drawTrack(initGPS,map)
    # move.followWall()
    #print(getOrientation())
    #move.turn90()
    map.detectWall(currentPos, 360-getOrientation())
    #print(beams[0])
    detect.run()
    #drawBeams(beams)
    #print((lidar.getRangeImage()))
    #aaa = lidar.getPointCloud()
    #print(aaa[1])
    print('----------------------------------------')
    pass


if __name__=='__main__':
    main()