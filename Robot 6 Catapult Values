#region VEXcode Generated Robot Configuration
from vex import *
import urandom

# Brain should be defined by default
brain=Brain()

# Robot configuration code
brain_inertial = Inertial()
catapult_motor_a = Motor(Ports.PORT12, False)
catapult_motor_b = Motor(Ports.PORT6, True)
catapult = MotorGroup(catapult_motor_a, catapult_motor_b)
touchled_2 = Touchled(Ports.PORT2)



# generating and setting random seed
def initializeRandomSeed():
    wait(100, MSEC)
    xaxis = brain_inertial.acceleration(XAXIS) * 1000
    yaxis = brain_inertial.acceleration(YAXIS) * 1000
    zaxis = brain_inertial.acceleration(ZAXIS) * 1000
    systemTime = brain.timer.system() * 100
    urandom.seed(int(xaxis + yaxis + zaxis + systemTime)) 
    
# Initialize random seed 
initializeRandomSeed()

#endregion VEXcode Generated Robot Configuration

# ------------------------------------------
# 
# 	Project:      VEXcode Project
# 	Author:       VEX
# 	Created:
# 	Description:  VEXcode IQ Python Project
# 
# ------------------------------------------

# Library imports
from vex import *

# Begin project code

catapult.set_velocity(100, PERCENT)

#scoring high goal - 100 degrees


def catapult_launch_degrees(degrees):
   
   catapult.spin_for(REVERSE, degrees, DEGREES)

def catapult_load_degree(degree):
   
   catapult.spin_for(FORWARD, degree, DEGREES)


#shoot high
catapult_launch_degrees(100)
wait(1, SECONDS)

#load for high goal from high
catapult_launch_degrees(800)
wait(1, SECONDS)

#shoot high
catapult_launch_degrees(100)
wait(1, SECONDS)

#load for low goal from high
catapult_load_degree(800)
wait(1, SECONDS)

#shoot low
catapult_launch_degrees(800)
wait(3, SECONDS)

#load for low goal from low
catapult_load_degree(800)
wait(1, SECONDS)

#shoot low 
catapult_launch_degrees(800)
wait(3, SECONDS)

#load for high goal from low
catapult_launch_degrees(810)


