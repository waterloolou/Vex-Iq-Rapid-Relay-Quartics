#region VEXcode Generated Robot Configuration
from vex import *
import urandom

# Brain should be defined by default
brain=Brain()

# Robot configuration code
brain_inertial = Inertial()
left_motor = Motor(Ports.PORT9, False)
right_motor = Motor(Ports.PORT3, True)
Intake_motor_a = Motor(Ports.PORT7, False)
Intake_motor_b = Motor(Ports.PORT1, True)
Intake = MotorGroup(Intake_motor_a, Intake_motor_b)
Catapult_motor_a = Motor(Ports.PORT12, False)
Catapult_motor_b = Motor(Ports.PORT6, True)
Catapult = MotorGroup(Catapult_motor_a, Catapult_motor_b)
touchled_2 = Touchled(Ports.PORT2)
Distance8 = Distance(Ports.PORT8)



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
# 	Project:      PID Test Control
# 	Author:       Louis Cottenier
# 	Created:      11/9/2024
# 	Description:  Testing PID control
# 
# ------------------------------------------

# Library imports
from vex import *

# Begin project code
start_intake = Event()
stop_intake = Event()
catapult_load = Event()
catapult_launch = Event()
error = 0
output = 0
error_total = 0
last_error = 0
derivative = 0
gear_ratio = 2 / 1


def move(left_speed, right_speed):
    left_motor.set_velocity(left_speed, PERCENT)
    right_motor.set_velocity(right_speed, PERCENT)
    left_motor.spin(FORWARD)
    right_motor.spin(FORWARD)


#(gear_ratio * 200)
#target distance is in MM
#speed is NOT velocity, speed affects velocity
#stuff is not workign and i am tired and broken :(
def drivestraitforward_PID(target_distance:int, speed:int, kp:float, ki:float, kd:float, target_heading:int):
    brain_inertial.set_rotation(0, DEGREES)
    left_motor.set_stopping(BRAKE)
    right_motor.set_stopping(BRAKE)
    left_motor.set_position(0, DEGREES)
    right_motor.set_position(0, DEGREES)
    error_total = brain_inertial.rotation() - target_heading
    while math.fabs(left_motor.position(DEGREES)) + math.fabs(right_motor.position(DEGREES)) / 2 < target_distance:
        error = brain_inertial.rotation() - target_heading
        last_error = error
        wait(500, MSEC)
        error = brain_inertial.rotation() - target_heading
        derivative = error - last_error
        # + 0.5
        output = error * kp + derivative * kd + (error_total * ki)
        move(speed - output, speed + output)
        error_total = error_total + error
    left_motor.stop()
    right_motor.stop()

def drivestraitbackward_PID(target_distance:int, speed:int, kp:float, ki:float, kd:float, target_heading:int):
    brain_inertial.set_rotation(0, DEGREES)
    left_motor.set_stopping(BRAKE)
    right_motor.set_stopping(BRAKE)
    left_motor.set_position(0, DEGREES)
    right_motor.set_position(0, DEGREES)
    error_total = brain_inertial.rotation() - target_heading
    while Distance8.object_distance(MM) > 50:
        print(Distance8.object_distance(MM))
        error = brain_inertial.rotation() - target_heading
        last_error = error
        wait(1, MSEC)
        error = brain_inertial.rotation() - target_heading
        derivative = error - last_error
        # + 0.5
        output = error * kp + derivative * kd + (error_total * ki)
        move(speed - output, speed + output)
        error_total = error_total + error
    left_motor.stop()
    right_motor.stop()


def turn_PID(target_distance:int, speed:float, kp:float, ki:float, kd:float, target_heading:int):
    brain_inertial.set_rotation(0, DEGREES)
    left_motor.set_stopping(BRAKE)
    right_motor.set_stopping(BRAKE)
    left_motor.set_position(0, DEGREES)
    right_motor.set_position(0, DEGREES)
    error_total = brain_inertial.rotation() - target_heading
    touchled_2.set_color(Color.RED)
    error = brain_inertial.rotation() - target_heading
    while math.fabs(error) > 4:
                
        brain.play_sound(SoundType.SIREN)
        error = brain_inertial.rotation() - target_heading
        last_error = error
        wait(20, MSEC)
        error = brain_inertial.rotation() - target_heading
        derivative = error - last_error
        # + 0.5
        output = error * kp + derivative * kd + (error_total * ki)
        move(speed - output, speed + output)
        error_total = error_total + error
    left_motor.stop()
    right_motor.stop()




#drivestrait_PID(1000, 62.5, 0.342, 0.02549, 0, 0)
#drivestrait_PID(1000, 62.5, 0.456, 0.05664596, 0.09177, 0)

#drivestrait_PID(1500, 62.5, 0.456, 0.05664596, 0.9177, 0)


#drivestrait_PID(1500, 62.5, 0.456, 0.05664596, 0.45885, 0)
#drivestraitbackward_PID(0, -62.5, 0.456, 0.05664596, 0.45885, 0)

'''
Move forward and backward 5 times code
#drivestrait_PID(1300, -62.5, 0.456, 0.05664596, 0.45885, 0)
#drivestrait_PID(1300, 62.5, 0.456, 0.05664596, 0.45885, 0)
#drivestrait_PID(1300, -62.5, 0.456, 0.05664596, 0.45885, 0)
#drivestrait_PID(1300, 62.5, 0.456, 0.05664596, 0.45885, 0)
#drivestrait_PID(1300, -62.5, 0.456, 0.05664596, 0.45885, 0)
#drivestrait_PID(1300, 62.5, 0.456, 0.05664596, 0.45885, 0)
#drivestrait_PID(1300, -62.5, 0.456, 0.05664596, 0.45885, 0)
#drivestrait_PID(1300, 62.5, 0.456, 0.05664596, 0.45885, 0)
#drivestrait_PID(1300, -62.5, 0.456, 0.05664596, 0.45885, 0)

'''

#Intake.spin_for(FORWARD, 900, DEGREES)


#drivestrait_PID(250, 62.5, 0.456, 0.05664596, 0.45885, -105)
#turn_PID(0, 50, 0.46, 0.05664596, 0.45885, 90)
1000/62.5
16
1.61
31

39.371

222
brain_inertial.set_rotation(0, DEGREES)
left_motor.set_stopping(BRAKE)
right_motor.set_stopping(BRAKE)
left_motor.set_position(0, DEGREES)
right_motor.set_position(0, DEGREES)
def turn(rotation1):
    error = brain_inertial.rotation(DEGREES) - rotation1 
    while math.fabs(error) > 1:
        if error < 0:  
            left_motor.spin(FORWARD)
            right_motor.spin(REVERSE)
        else:
            left_motor.spin(REVERSE)
            right_motor.spin(FORWARD)   

        wait(1, MSEC)
    right_motor.stop()
    left_motor.stop()

#turn(90)






