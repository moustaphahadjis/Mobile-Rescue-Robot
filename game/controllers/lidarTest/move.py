import math
import time

class Move:
    def __init__(self, robot, timestep, lasers):
        self.gps = robot.getDevice('gps')
        self.gps.enable(timestep)
        self.timestep = timestep
        self.leftMotor = robot.getDevice('left wheel motor')
        self.rightMotor = robot.getDevice('right wheel motor')

        self.max_speed = 6.28  # Adjust as needed

        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))

        self.iunit = robot.getDevice('iunit')
        self.iunit.enable(timestep)
        self.robot = robot
        self.lasers = lasers
        self.distThr = 0.05

    def forward(self):
        self.leftMotor.setVelocity(self.max_speed)
        self.rightMotor.setVelocity(self.max_speed)


    def backward(self):
        self.leftMotor.setVelocity(-self.max_speed)
        self.rightMotor.setVelocity(-self.max_speed)


    def left(self):
        self.leftMotor.setVelocity(-self.max_speed/2)
        self.rightMotor.setVelocity(self.max_speed/2)

    def right(self):
        self.leftMotor.setVelocity(self.max_speed/2)
        self.rightMotor.setVelocity(-self.max_speed/2)
    
    def stop(self):
        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)

    def getOrientation(self):
        yaw = self.iunit.getRollPitchYaw()[2]

        return math.degrees(yaw)%360
    

    
    def getBearing(self, lat1, long1, lat2, long2):
        lat1 = math.radians(lat1)
        long1 = math.radians(long1)
        lat2 = math.radians(lat2)
        long2 = math.radians(long2)
        
        # Calculate the bearing
        rad = math.atan2(
            math.sin(long2 - long1) * math.cos(lat2),
            math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(long2 - long1)
        )
        
        # Convert the bearing to degrees
        bearing = math.degrees(rad)
        
        # Make sure the bearing is positive
        bearing = (bearing + 360) % 360
        return bearing
        #return bearing
        
    
    def turn90(self):
        self.stop()
        curr = self.getOrientation()
        print(curr)
        target = self.getOrientation() + 90
        if target>360:
            target= target - 360
        
        print(f'target:{target},')
        while self.robot.step(self.timestep) !=1:
            self.left()
            print(f'Orientation:{self.getOrientation()},')
            if self.getOrientation()-target<=0:
                self.stop()
                break
            if self.getOrientation() == curr:
                break
        return True
    
    def followWall(self):
            max_speed = 6.28
            rightWall = self.lasers[0].getValue()< self.distThr 
            cornerWall = self.lasers[2].getValue()< self.distThr
            frontWall = self.lasers[2].getValue()< self.distThr or self.lasers[3].getValue()< self.distThr
            
            print(f'Left:{self.lasers[6].getValue()}, Corner:{self.lasers[4].getValue()},front{self.lasers[3].getValue()}')
            
            
            left_speed = max_speed
            right_speed = max_speed

            if frontWall:
                print("R")
                left_speed = -max_speed
                right_speed = max_speed
            else:
                if rightWall:
                    print("F")
                    left_speed = max_speed
                    right_speed = max_speed
                else:
                    print("L")
                    left_speed = max_speed
                    right_speed = -0
                if cornerWall:
                    print("Adjust Space")
                    left_speed = -0
                    right_speed = max_speed
            
            self.leftMotor.setVelocity(left_speed)
            self.rightMotor.setVelocity(right_speed)

    def isFrontFree(self):
        thr = 0.6
        print(self.lasers[3].getValue() )
        if self.lasers[3].getValue() > thr and self.lasers[4].getValue() > thr:
            return True
        else:
            print('Obstacle Front')
            return False
        
    def isRight(self):
       thr = 0.08
       if self.lasers[0].getValue()>thr or self.lasers[1].getValue()>thr or self.lasers[2].getValue()>thr :
            print('right free')
            return True
       else:
            print('Obstacle Right')
            return False
       
    def isLeft(self):
       thr = 0.08
       if   self.lasers[5].getValue()>thr or self.lasers[6].getValue()>thr or self.lasers[7].getValue()>thr:
            print('left free')
            return True
       else:
            print('Obstacle Left')
            return False
       
    def goLeft(self):
            angle = 0
            if(not(self.getOrientation()>angle-2 and self.getOrientation()<angle+2)):
                if(self.getOrientation()>0 and self.getOrientation()<180):
                    self.right()
                else:
                    self.left()
            else:
                self.forward()
    def goRight(self):
            angle = 180
            if(not(self.getOrientation()>angle-2 and self.getOrientation()<angle+2)):
                if(self.getOrientation()>180 and self.getOrientation()<360):
                    self.right()
                else:
                    self.left()
            else:
                self.forward()
    def goUP(self):
            orient = self.getOrientation()
            angle = 270
            if(not(self.getOrientation()>angle-2 and self.getOrientation()<angle+2)):
                if(orient>=0 and orient<90) or (orient>270 and orient<=360):
                    self.right()
                else:
                    self.left()
            else:
                self.forward()
    def goDown(self):
            orient = self.getOrientation()
            angle = 90
            if(not(self.getOrientation()>angle-2 and self.getOrientation()<angle+2)):
                if(orient>90 and orient<=180) or (orient>180 and orient<=270):
                    self.right()
                else:
                    self.left()
            else:
                self.forward()

       
    def faceDir(self, gps2):
        gps1 = self.gps.getValues()
        bearing = self.getBearing(gps1[0], gps1[1], gps2[0], gps2[1])
        orientation = self.getOrientation()
        if not(bearing > orientation +5 and bearing <orientation - 5):
            while self.robot.step(self.timestep) != -1:
                bearing = self.getBearing(gps1[0], gps1[1], gps2[0], gps2[1])
               # print(f'bearing:{bearing}, orien:{self.getOrientation()}')
                if bearing>self.getOrientation()+5:
                    self.left()
                elif bearing<self.getOrientation()-5:
                    self.right()
                else:
                    self.stop()
                    break
                
            return True
        else:
            return False
    def r_obstacle (self):
        rr = sum((self.lasers[i].getValue() for i in range(0,3)))/3
        print(rr)
        return rr
    def l_obstacle (self):
        rr = sum((self.lasers[i].getValue() for i in range(4,7)))/3
        print(rr)
        return rr
    
    def rotate(self, angle, map):
        val = False

        r = angle%360
        o = self.getOrientation()%360

        dif = (r-o)%360
        #print(dif)
        
        while self.robot.step(self.timestep)!=-1:
            cur = self.getOrientation()
            map.mapping(cur, self.gps.getValues())
            if cur < angle+3 and cur > angle -3:
                val = True
                self.stop()
                #print('stop')
                break
            else:
                if dif < 180:
                    self.left()
                else :
                    self.right()
        
        return val
    def getAngle(self, next, curr):
        x = next[0]
        y = next[1]
        x0 =curr[0]
        y0 = curr[1]
        
        a = 0
        if x < x0 and y< y0:
            a = 315
        elif x == x0 and y<y0:
            a =270
        elif x>x0 and y<y0:
            a = 225
        elif x<x0 and y == y0:
            a = 0
        elif x >x0 and y==y0:
            a = 180
        elif x<x0 and y>y0:
            a = 45
        elif x == x0 and y>y0:
            a = 90
        elif x>x0 and y>y0:
            a = 135
        else:
            a = -1
        return a

    def moveTo(self, map, next):
        map.mapping(self.getOrientation(), self.gps.getValues())
        dist = self.getDist(next,map.curr)
        if dist>0.9:
            angle = self.getAngle(next, map.curr)
            print(f'angle: {angle}, dist: {dist}')
            if self.rotate(angle, map):
                self.forward()
            return False
        else:
            return True


    def goTo(self,gps2, map, path):

        gps1 = self.gps.getValues()
        bearing = self.getBearing(gps1[0], gps1[2], gps2[0], gps2[1]) 
        
        #print(f'gps1={gps1}, gps2={gps2}')
        if bearing<0:
            bearing = 360 - bearing
        
        val = 0
        
        while self.robot.step(self.timestep)!=-1:

            dist = self.getDist(self.gps.getValues(), gps2)
            #print(f'Dist ={dist}')
            if dist<0.1:
                print('breaking')
                self.stop()
                val = 1
                
            
            #bearing = self.getBearing(gps1[0], gps1[1], gps2[0], gps2[1])
        # print(f'bearing:{bearing}, orien:{self.getOrientation()}')
            orientation = self.getOrientation() 
            map.mapping(orientation, self.gps.getValues())
            print(f'o:{orientation}, b:{bearing}')
            if not map.securePath(path):
                val = 2
            if bearing>orientation+2 or bearing<orientation-2:
            # print('3')
                self.right()
            else:
                #print('5555')
                self.forward()



        
        print(f'-----{val}')
        return val
    def startMapping(self,map):
        map.mapping(self.getOrientation() , self.gps.getValues())
    def getDist(self, start, end):
       a = -start[0] + end[0]
       b = -start[1] + end[1]

       return math.sqrt(a**2 + b**2)



                
            
    

    
