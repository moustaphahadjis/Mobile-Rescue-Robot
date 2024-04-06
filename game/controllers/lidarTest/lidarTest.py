"""lidarTest controller."""
import math
import numpy as np
from move import Move
from map import Map

from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

lidar = robot.getDevice('lidar')
lidar.enable(timestep)
lidar.enablePointCloud()

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


#Get angle of each Beam
#totalAngle = 6.283
totalAngle = 360
numBeams = 10
anglePerBeam = totalAngle/numBeams

def getBeams(robotOrientation, values, currentPos):
    result = np.zeros((numBeams,3))
    #i = numBeams-1
    #while i >=0:
    for i in range(numBeams):
        angle = (i*anglePerBeam) + robotOrientation
        if angle>360:
            angle = angle-360
        
        y = values[i] * math.sin(angle) 
        x = values[i] * math.cos(angle)

        
        result[i][0] = x*displayRes*0.5 + currentPos[0] if values[i]<0.8 else 0
        result[i][1] = -y*displayRes*0.5 + currentPos[1] if values[i]<0.8 else 0
        result[i][2] = (values[i])
        
    
    return result

def drawBeams(points,map):
    for i in range(numBeams):
        if(points[i][0] !=0 and points[i][1]!=0):
            #display.drawPixel(points[i][0],points[i][1])
            map.setWall(points[i][0],points[i][1])

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
 move = Move(robot,)
 map = Map(robot,displayRes)

 robot.step(1000)
 initGPS = gps.getValues()
 currentPos = (0,0)
 while robot.step(timestep) != -1:
    vals = lidar.getRangeImage()
    print(  gps.getValues())
    beams = getBeams(getOrientation(),vals, currentPos)
    drawBeams(beams,map)
    currentPos = drawTrack(initGPS,map)
    move.forward()
    #print(beams[0])
    
    #drawBeams(beams)
    #print((lidar.getRangeImage()))
    #aaa = lidar.getPointCloud()
    #print(aaa[1])
    print('----------------------------------------')
    pass


if __name__=='__main__':
    main()