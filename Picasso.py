import umath

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch


hub = PrimeHub()
A_motor=Motor(Port.F,Direction.CLOCKWISE)
E_motor=Motor(Port.E,Direction.CLOCKWISE)
foot_motor=Motor(Port.D,Direction.COUNTERCLOCKWISE)


def cosLaw(A, B, C):
    return umath.acos((A**2 - B**2 - C**2) / (-2 * B * C))

class Picasso:
    def __init__(self,base_length,a_length,b_length,c_length,d_length,Gear_Ratio_1,Gear_Ratio_2,x0,y0):
        self.a = a_length
        self.b = b_length
        self.c = c_length
        self.d = d_length
        self.e = base_length
        self.gr1=Gear_Ratio_1
        self.gr2=Gear_Ratio_2
        self.x0=x0
        self.y0=y0

    def inverseKinematics(self, xd, yd):
        try:
            # Compute distances
            dist = umath.sqrt(yd**2 + xd**2)
            distR = umath.sqrt(yd**2 + (self.c - xd)**2)

            # Calculate possible solutions for alpha and beta
            alpha1 = 360 + umath.atan2(yd, xd) * 180 / umath.pi - cosLaw(self.b, self.a, dist) * 180 / umath.pi
            beta1 = 180 - cosLaw(dist, self.a, self.b) * 180 / umath.pi

            alpha2 = 360 + umath.atan2(yd, xd) * 180 / umath.pi + cosLaw(self.b, self.a, dist) * 180 / umath.pi
            beta2 = 180 + cosLaw(dist, self.a, self.b) * 180 / umath.pi

            # Calculate possible solutions for gamma and epsilon
            gamma1 = 180 + cosLaw(distR, self.a, self.b) * 180 / umath.pi
            epsilon1 = 180 - umath.atan2(yd, (self.c - xd)) * 180 / umath.pi + cosLaw(self.b, self.a, distR) * 180 / umath.pi

            gamma2 = 180 - cosLaw(distR, self.a, self.b) * 180 / umath.pi
            epsilon2 = 180 - umath.atan2(yd, (self.c - xd)) * 180 / umath.pi - cosLaw(self.b, self.a, distR) * 180 / umath.pi

            # Select the second solution that avoids crossing the linkage
            # This logic assumes "crossing" is determined by angles in specific ranges
            if alpha1 < alpha2:  # Example condition; adjust based on your linkage geometry
                alpha, beta = alpha2, beta2
            else:
                alpha, beta = alpha1, beta1

            if gamma1 > gamma2:  # Example condition; adjust based on your linkage geometry
                gamma, epsilon = gamma2, epsilon2
            else:
                gamma, epsilon = gamma1, epsilon1

            return (alpha, beta, gamma, epsilon)
        except Exception as e:
            print(e)
            return (None, None, None, None)
        
    def InterpolateMoves(self,points, stepsizecm=0.1):
        points.insert(0,[self.x0,self.y0,False])
        path_plan = []  # To store the result

        for i in range(len(points) - 1):
            x0, y0, state0 = points[i]
            x1, y1, state1 = points[i + 1]

            # Calculate the distance between the points
            dx = x1 - x0
            dy = y1 - y0
            distance = umath.sqrt(dx**2 + dy**2)

            # Calculate the number of steps based on the step size
            num_steps = int(distance / stepsizecm)

            # Generate interpolated points
            if num_steps>0:
                for step in range(num_steps + 1):
                    t = step / num_steps  # Interpolation factor (0 to 1)
                    xi = x0 + t * dx
                    yi = y0 + t * dy

                    # Use the state of the starting point for the segment
                    path_plan.append([xi, yi, state0])

        # Add the last point explicitly to ensure it is included
        path_plan.append(points[-1])

        return path_plan
    
    def plot(self,path_plan):

        A_motor.reset_angle(90*self.gr1)
        E_motor.reset_angle(90*self.gr2)
        foot_motor.reset_angle(0)

        for i, point in enumerate(path_plan):
            xd=point[0]
            yd=point[1]
            state=point[2]
            alpha,beta,gamma,epsilon = self.inverseKinematics(xd,yd)
            alpha-=360
            #print((alpha,A_motor.angle()),(epsilon,E_motor.angle()))
            if state:
                TurnMotorForAngle(foot_motor,0,threshold=1,Kp=3,Ki=1/100)
            else:
                TurnMotorForAngle(foot_motor,40,threshold=1,Kp=3,Ki=1/100)

            TurnMotorsForAngle(A_motor,E_motor,alpha*self.gr1,epsilon*self.gr2,threshold=1,Kp=3,Ki=1/1000)

            #TurnMotorForAngle(A_motor,alpha*self.gr1,threshold=1,Kp=3,Ki=1/1000)
            #TurnMotorForAngle(E_motor,epsilon*self.gr2,threshold=1,Kp=3,Ki=1/1000)

    

def TurnMotorForAngle(desired_motor,desired_angle,threshold=1,Kp=3,Ki=1/1000):
    integral_error = 0  # Initialize the integral error

    while (abs(desired_angle-desired_motor.angle()) > threshold):
        logs=((desired_angle,desired_motor.angle()))
        print(logs)

        # Calculate the current error
        error = desired_angle - desired_motor.angle()
            
        # Accumulate the error for the integral term
        integral_error += error

        desired_motor.run(Kp*(desired_angle-desired_motor.angle())+Ki*integral_error)

    desired_motor.stop()

def TurnMotorsForAngle(desired_motor1,desired_motor2,desired_angle1,desired_angle2,threshold=1,Kp=3,Ki=0):
    integral_error1 = 0  # Initialize the integral error
    integral_error2 = 0  # Initialize the integral error

    while (abs(desired_angle1-desired_motor1.angle()) > threshold) and (abs(desired_angle2-desired_motor2.angle()) > threshold):
        logs=((desired_angle1,desired_motor1.angle()),(desired_angle2,desired_motor2.angle()))
        print(logs)
        # Calculate the current error
        error1 = desired_angle1 - desired_motor1.angle()
        error2 = desired_angle2 - desired_motor2.angle()
            
        # Accumulate the error for the integral term
        integral_error1 += error2
        integral_error2 += error1


        desired_motor1.run(Kp*(desired_angle1-desired_motor1.angle())+Ki*integral_error1)
        desired_motor2.run(Kp*(desired_angle2-desired_motor2.angle())+Ki*integral_error2)

    desired_motor1.stop()
    desired_motor2.stop()


#TurnMotorForAngle(A_motor,90*3,Kp=3,Ki=1/1000)