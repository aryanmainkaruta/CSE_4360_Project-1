from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()

ultra=UltrasonicSensor(Port.D)
hub = PrimeHub()
LeftMotor=Motor(Port.A,Direction.CLOCKWISE)
RightMotor=Motor(Port.B,Direction.COUNTERCLOCKWISE)
SweepMotor=Motor(Port.F,Direction.CLOCKWISE)
button = ForceSensor(Port.E)


multiplier=1.5
wallHuggingDistance=120
basePower=100

def sweepUP():
    SweepMotor.run(-1000)

def sweepDown():
    SweepMotor.run(1000)

def ceiling(commandedValue,maxedValue,minValue):
    if commandedValue > maxedValue:
        return maxedValue
    if commandedValue < minValue:
        return minValue
    return commandedValue

def TurnForAngle(desired_angle,threshold=1,Kp=3):
    if hub.imu.ready():
        hub.imu.reset_heading(0)
        while (abs(desired_angle-hub.imu.heading()) > threshold) and (abs(desired_angle-hub.imu.heading()) > threshold):
            LeftMotor.run(-Kp*(desired_angle-hub.imu.heading()))
            RightMotor.run(Kp*(desired_angle-hub.imu.heading()))
            print(desired_angle-hub.imu.heading())
        print("Finished Move")
        LeftMotor.stop()
        RightMotor.stop()
    else:
        print("Hub Not Ready")

def DriveTrain(correctionValue):
    offset=ceiling(correctionValue,25,-25)

    RightMotor.run(basePower-offset)
    LeftMotor.run(basePower+offset)

sweepUP()
wait(1000)
SweepMotor.brake()


# def sweepForWall():

#     closest_distance = float('inf')  # Start with a very large distance
#     closest_angle = 0  # To store the angle where the closest wall was detected
#     initial_angle = gyro.angle()  # Get the initial angle

#     # Begin 360-degree sweep
#     while abs(gyro.angle() - initial_angle) < 360:
#         SweepMotor.run(50)  # Run the motor to turn the robot

#         # Check distance with ultrasonic sensor
#         current_distance = ultra.distance()

#         # If the current distance is less than the closest found so far, update closest
#         if current_distance < closest_distance:
#             closest_distance = current_distance
#             closest_angle = gyro.angle()

#     # Stop turning after completing the 360-degree sweep
#     SweepMotor.brake()

#     # Turn to the angle where the closest wall was detected
#     TurnForAngle(closest_angle - gyro.angle())

while True:
    DriveTrain(ultra.distance()-wallHuggingDistance)
    if(button.pressed()):
        # Back up and Turn other Way
        print("BACKUPPP")
        RightMotor.run(-50)
        LeftMotor.run(-50)
        wait(1000)
        TurnForAngle(90)
        
        
# def extinguish_fire():        
#     print("Activating fan to extinguish fire...")
#     # Run fan motor for a set duration
#     fan_motor.run(1000)  # Run fan at full speed
#     wait(3000)  # Run fan for 3 seconds
#     fan_motor.stop()  # Stop the fan after extinguishing
#     print("Fire extinguished.")

# def detect_fire():
#     # Scan for the color of the fire (simulated by detecting a specific color or intensity)
#     if color_sensor.color() == Color.RED:  # Assuming red paper represents fire
#         print("Fire detected!")
#         # Raise an alarm (could be visual or audible)
#         hub.speaker.beep()  # Beep to indicate fire detection
#         return True
#     else:
#         return False




