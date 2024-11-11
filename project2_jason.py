from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()

ultra = UltrasonicSensor(Port.D)
LeftMotor = Motor(Port.A, Direction.CLOCKWISE)
RightMotor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
SweepMotor = Motor(Port.F, Direction.CLOCKWISE)
button = ForceSensor(Port.E)

multiplier = 1.5
wallHuggingDistance = 120
basePower = 100

# Timer for periodic inward turns
timer = StopWatch()
spiral_interval = 5000  # Time in milliseconds to make an inward turn (adjust as needed)

def sweepUP():
    SweepMotor.run(-1000)

def sweepDown():
    SweepMotor.run(1000)

def ceiling(commandedValue, maxedValue, minValue):
    if commandedValue > maxedValue:
        return maxedValue
    if commandedValue < minValue:
        return minValue
    return commandedValue

def TurnForAngle(desired_angle, threshold=1, Kp=3, relative=False):
    """
    Turns the robot by a specified angle.
    If `relative` is True, turns by the desired_angle from the current heading.
    """
    if hub.imu.ready():
        if relative:
            desired_angle += hub.imu.heading()  # Make the turn relative to the current heading
        hub.imu.reset_heading(0)
        
        while abs(desired_angle - hub.imu.heading()) > threshold:
            error = desired_angle - hub.imu.heading()
            LeftMotor.run(-Kp * error)
            RightMotor.run(Kp * error)
            print("Turning, error:", error)
        
        print("Finished Turn")
        LeftMotor.stop()
        RightMotor.stop()
    else:
        print("Hub Not Ready")

def DriveTrain(correctionValue):
    offset = ceiling(correctionValue, 25, -25)
    RightMotor.run(basePower - offset)
    LeftMotor.run(basePower + offset)

sweepUP()
wait(1000)
SweepMotor.brake()

while True:
    # Wall-following behavior
    DriveTrain(ultra.distance() - wallHuggingDistance)

    # Check if the button is pressed (obstacle avoidance)
    if button.pressed():
        print("BACKUPPP")
        RightMotor.run(-50)
        LeftMotor.run(-50)
        wait(1000)
        TurnForAngle(90, relative=True)  # Turn 90 degrees relative to the current heading

    # Periodic inward turn to create a spiral movement
    if timer.time() > spiral_interval:
        timer.reset()  # Reset the timer for the next interval
        TurnForAngle(15, relative=True)  # Turn 15 degrees inward (adjust angle as needed)
