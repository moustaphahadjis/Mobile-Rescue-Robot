class Move:
    def __init__(self, robot):
        self.leftMotor = robot.getDevice('left wheel motor')
        self.rightMotor = robot.getDevice('right wheel motor')

        self.max_speed = 6.28  # Adjust as needed

        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))

    def forward(self):
        self.leftMotor.setVelocity(self.max_speed)
        self.rightMotor.setVelocity(self.max_speed)

# Function to move the robot backward
    def backward(self):
        self.leftMotor.setVelocity(-self.max_speed)
        self.rightMotor.setVelocity(-self.max_speed)


    def left(self):
        self.leftMotor.setVelocity(-self.max_speed)
        self.rightMotor.setVelocity(self.max_speed)

# Function to turn the robot right
    def right(self):
        self.leftMotor.setVelocity(self.max_speed)
        self.rightMotor.setVelocity(-self.max_speed)

