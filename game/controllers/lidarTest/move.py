import math
import time
import random

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
    def slow_forward(self):
        self.leftMotor.setVelocity(self.max_speed/5)
        self.rightMotor.setVelocity(self.max_speed/5)


    def backward(self):
        self.leftMotor.setVelocity(-self.max_speed)
        self.rightMotor.setVelocity(-self.max_speed)


    def left(self):
        self.leftMotor.setVelocity(-self.max_speed/4)
        self.rightMotor.setVelocity(self.max_speed/4)

    def right(self):
        self.leftMotor.setVelocity(self.max_speed/4)
        self.rightMotor.setVelocity(-self.max_speed/4)
    
    def slow_left(self, map):
        self.leftMotor.setVelocity(-self.max_speed/5)
        self.rightMotor.setVelocity(self.max_speed/5)
        self.startMapping(map)

    def slow_right(self, map):
        self.leftMotor.setVelocity(self.max_speed/5)
        self.rightMotor.setVelocity(-self.max_speed/5)
        self.startMapping(map)
    
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
        thr = 0.1
        #print(self.lasers[8].getValue() )
        if self.lasers[8].getValue() > thr :
            return True
        else:
            #print('Obstacle Front')
            return False
        
    def isRight(self):
       thr = 0.2
       if self.lasers[0].getValue()>thr :
            #print('right free')
            return True
       else:
            #print('Obstacle Right')
            return False
       
    def isLeft(self):
       thr = 0.2
       if   self.lasers[7].getValue()>thr :
            #print('left free')
            return True
       else:
            #print('Obstacle Left')
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
    
    def rotate(self, angle,map, detect):
        val = False

        r = angle%360
        o = self.getOrientation()%360

        dif = (r-o)%360
        #print(dif)
        #print(f'angle:{angle}')
        
        while self.robot.step(self.timestep)!=-1:
            self.startMapping(map)
            detect.run()
            cur = self.getOrientation()
            #print(f'curr={cur}')
            #map.mapping(cur, self.gps.getValues())
            if cur < angle + 3 and cur > angle - 3:
                val = True
                #self.stop()
                #print('Roration done')
                break
            else:
                if dif < 180:
                    self.left()
                    val = False
                else :
                    self.right()
                    val = False
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

    def moveTo(self, map, next, path):
        print('moving')
        val = False
        map.mapping(self.getOrientation(), self.gps.getValues())
        dist = self.getDist(next,map.curr)
        if dist>2:
            angle = self.getAngle(next, map.curr)
            print(f'angle: {angle}, dist: {dist}')
            if self.rotate(angle, map):
                self.forward()
            val = False
        else:
            val = True
        
        return val


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
    def startMapping(self,map, ):
        map.mapping(self.getOrientation() , self.gps.getValues())
    def getDist(self, start, end):
       a = -start[0] + end[0]
       b = -start[1] + end[1]

       return math.sqrt(a**2 + b**2)
    
    def tremaux(self, map,detect):
        visited = dict()
        canDetect = True
        lastDec = ''
        while self.robot.step(self.timestep) != -1:
            """
            t1 = time.time()
            if canDetect:
                if detect.hasDetected():
                    t2 = time.time()
                    if(t2-t1)>10:
                        canDetect = False
                    self.stop()
                    detect.run()
               """
            signs,floor = detect.run()     

                    
            self.startMapping(map)
            position = map.curr
            if position not in visited:
                visited[position] = 11
            visited[position] += 10
            
            if visited[position] > 10 :
                #print(floor)
                poss = []
                if self.isFrontFree() and floor!=2:
                    poss.append('f')
                    if(lastDec=='f'):
                        if self.isRight():
                            poss.append('r')
                        if self.isLeft():
                            poss.append('l')
  
                    dec = ['f']
                    if len(poss)==1:
                        dec = poss
                    elif len(poss)==2:
                        dec = random.choices(poss, weights=(80,20))
                    
                    elif len(poss)==2:
                        dec = random.choices(poss, weights=(80,10,10))

                    
                    
                    if dec[0] == 'f':
                        
                        if floor == 1:
                            self.slow_forward()
                        else:
                            self.forward()
                                        
                    else:     
                        angle = 0   
                        if dec[0] == 'l' :
                            angle = self.getOrientation()+90
                        elif dec[0] =='r':
                            angle = self.getOrientation()-90
                        if angle > 360:
                            angle = angle-360
                        if angle < 0:
                            angle = 360 + angle
                            
                        if self.rotate(angle,map, detect):
                            self.forward
                    
                    
                else:
                    # Turn randomly
                    rand = random.Random(2)
                    angle = 0
                    
                    if rand ==0:
                        angle = self.getOrientation()+90
                    else:
                        angle = self.getOrientation()-90

                    

                    if angle > 360:
                        angle = angle-360
                    if angle < 0:
                        angle = 360 + angle
                    
                    if self.rotate(angle,map, detect):
                        self.stop()

                    """
                if self.isFrontFree() and floor!=2:
                    if floor == 1:
                        self.slow_forward()
                    else:
                        self.forward()
                    canDetect = True """
            else:
                # Backtrack or turn around if this path is highly visited
                angle = self.getOrientation() - 180
                if angle>360:
                    angle = angle-360
                if angle<0:
                    angle = 360 + angle
                
                if self.rotate(angle,map,detect):
                    self.stop()


                
            
    

    
