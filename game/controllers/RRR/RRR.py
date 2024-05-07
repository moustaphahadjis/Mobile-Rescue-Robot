"""RRR controller."""
import math
import numpy as np
from move import Move
from map import Map
from detection import Detection
from explore import Explore
import time
from controller import Supervisor

robot = Supervisor()
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

def xyToGps(initGPS, x,y):
    x0 = (x - pos[0])/(scale*displayRes)
    y0 = (y - pos[1])/(scale*displayRes)

    x1 = -x0 + initGPS[0]
    y1 = -y0 + initGPS[2]

    return (y1,x1)

def drawTrack(initGPS, map):
    x0 = -gps.getValues()[0] + initGPS[0]
    y0 = -gps.getValues()[2] + initGPS[2]

    x = x0*displayRes*scale + pos[0]
    y = y0*displayRes*scale + pos[1]
    map.setTrack(x,y)
    return (x,y)
    #print(f'{x,y}')


iunit = robot.getDevice('iunit')
iunit.enable(timestep)
def main():
 
 robot.step(timestep)
 initGPS = gps.getValues()
 map = Map(robot,timestep,initGPS)
 move = Move(robot,timestep, map.lasers)
 detect = Detection(robot, timestep,move, map)
 #explore = Explore()

 
 makePath  = True
 pos = 0
 path=()
 next=[]
 t1 = time.time()

 #move.startMapping(map)
 while robot.step(timestep)!=-2:
    move.tremaux(map,detect)
    #detect.run()
    if False:

  
        if makePath:
            
            print('new path')

            robot.step(timestep)
            next = (explore.getEnd(map))

            path = explore.pathFinder(map.world, map.curr, (next[0], next[1]))

            if not map.securePath(path):
                print(f'Next: {next}')
                makePath = False
                map.setPath(path)
                t1  = time.time()
                print('----------------------------------------')
        else:
            
            #while robot.step(timestep)!=-1:
            con = True
            t2 = time.time()
            print(t2-t1)

            if(t2-t1)>5 or not map.securePath(path):
                makePath = True
                con = False
                print('Time Out')
                
            if con:
                if move.moveTo(map, next,path):
                    map.removePath(path)
                    makePath = True
            else:
                makePath = True
                print('Break')
        pass
    
  


if __name__=='__main__':
    main()
    print('end')