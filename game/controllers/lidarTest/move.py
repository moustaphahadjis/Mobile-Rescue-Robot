import math

class Move:
    def __init__(self, robot, timestep, lasers):
        self.timestep = timestep
        self.leftMotor = robot.getDevice('left wheel motor')
        self.rightMotor = robot.getDevice('right wheel motor')

        self.max_speed = 6.28  # Adjust as needed

        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))

        self.compass = robot.getDevice('compass')
        self.compass.enable(timestep)
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
        rad = math.atan2(self.compass.getValues()[1], self.compass.getValues()[0])
        angle = (rad - 1.5708) / 3.14159 * 180.0
        if (angle < 0.0):
            angle = angle + 360.0
        return angle
    
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
            

