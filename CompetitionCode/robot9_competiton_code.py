#region VEXcode Generated Robot Configuration
from vex import *
import urandom

# Brain should be defined by default
brain=Brain()

# Robot configuration code
brain_inertial = Inertial()
left_motor = Motor(Ports.PORT1, False)
right_motor = Motor(Ports.PORT7, True)
intake_outake_motor_a = Motor(Ports.PORT6, True)
intake_outake_motor_b = Motor(Ports.PORT5, False)
intake_outake = MotorGroup(intake_outake_motor_a, intake_outake_motor_b)
touchled_4 = Touchled(Ports.PORT4)
distance_3 = Distance(Ports.PORT3)
catapult = Motor(Ports.PORT12, True)
intake = Motor(Ports.PORT11, False)
bumper_8 = Bumper(Ports.PORT8)



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


# register for intake and catapult events
forward_intake = Event()
reverse_intake = Event()
low_high = Event()
high_max_height = Event()
high_low = Event()
stop_intake_and_outake = Event()
# code for making left and right motors as drive train so they move together
def move(left_speed, right_speed):
    left_motor.set_velocity(left_speed, PERCENT)
    right_motor.set_velocity(right_speed, PERCENT)
    left_motor.spin(FORWARD)
    right_motor.spin(FORWARD)


#PID Calculation
#---------------------------------------------------------------------------------------------------------------------------------------------------------------
def drivestraitforward_PID(target_distance:int, speed:int, kp:float, ki:float, kd:float, target_heading:int):
    speed = 100
    left_motor.set_position(0, DEGREES)
    right_motor.set_position(0, DEGREES)

    #error_total is used for K(I)
    error_total = brain_inertial.rotation() - target_heading

    # based on the current heading find the change in heading needed to achieve target heading
    startdistance = distance_3.object_distance(MM)
    distance_moved = distance_3.object_distance(MM) - startdistance

    # error_total is used for K(I)
    while (distance_moved < target_distance - 15 and (math.fabs(left_motor.position(DEGREES)) + math.fabs(right_motor.position(DEGREES))) / 2 < target_distance):
        distance_moved = distance_3.object_distance(MM) - startdistance
        error = brain_inertial.rotation() - target_heading
        last_error = error
        wait(10, MSEC)
        error = brain_inertial.rotation() - target_heading
        # error_total is used for K(D)
        derivative = error - last_error

        # caculate velocity offset based on k(P), K(I) and K(D)
        output = error * kp + derivative * kd + (error_total * ki)

        move(speed - output, speed + output)
        error_total = error_total + error

    left_motor.stop()
    right_motor.stop()
    #print("Ending heading forward: ", brain_inertial.rotation())
    #print("End Error total: ", error_total)
    #print("Distance moved: ", distance_moved)

def drivestraitbackward_PID(target_distance:int, speed:int, kp:float, ki:float, kd:float, target_heading:int):
    left_motor.set_position(0, DEGREES)
    right_motor.set_position(0, DEGREES)
    speed = -100
    # based on the current heading find the change in heading needed to achieve target heading
    # error_total is used for K(I)
    error_total = brain_inertial.rotation() - target_heading
    startdistance = distance_3.object_distance(MM)
    distance_moved = startdistance - distance_3.object_distance(MM)
    print("Start distance for backward ",startdistance)

    while (distance_3.object_distance(MM) > 15 and (math.fabs(left_motor.position(DEGREES)) + math.fabs(right_motor.position(DEGREES))) / 2 < target_distance): #or distance_moved < target_distance:
        distance_moved = distance_3.object_distance(MM) - startdistance

        error = brain_inertial.rotation() - target_heading
        last_error = error
        wait(10, MSEC)
        error = brain_inertial.rotation() - target_heading
        #print("Error", error)
        # error_total is used for K(D)
        derivative = error - last_error

        # caculate velocity offset based on k(P), K(I) and K(D)
        output = error * kp + derivative * kd + (error_total * ki)

        #print("Output Backwards: ", output)
        move(speed - output, speed + output)
        error_total = error_total + error

    move(speed, speed)
    left_motor.stop()
    right_motor.stop()
    print("Ending heading backward: ", brain_inertial.rotation())
    print("End Error total: ", error_total)
    print("Distance moved: ", distance_moved)

'''
We take in the parameters of speed for how fast we want to go.
kp, ki, and kd are values that affect the output (how it turns)
The target heading tells us where we want to go.
kp is the rate of change that the robot moves.
ki takes the past error adds it up, and is used to change the rate of change
kd uses that past to predicte future outcomes, and changes the rate of change
We use print statements and led for debuging all throughout the code. This helps us tune the code to work.
'''
def turn_PID(speed:float, kp:float, ki:float, kd:float, target_heading:int):
    start_time3 = brain.timer.time(MSEC)
    
    # Setting the rotation and position to 0 as a starting point
    # Setting the motors to brake so that they don't drift.
    #brain_inertial.set_rotation(0, DEGREES)
    left_motor.set_position(0, DEGREES)
    right_motor.set_position(0, DEGREES)
    print("Starting heading turn: ", brain_inertial.rotation())

    # Calculating error total for ki
    error_total = target_heading - brain_inertial.rotation()
    touchled_4.set_color(Color.RED)

    # Setting the error for a starting value.
    error = target_heading - brain_inertial.rotation()
    derivative = 0
    no_error_count = 0
    
    # Loop that moves robot until there is no error
    while math.fabs(error) > .75 or no_error_count <= 2:
        print(brain.timer.time(MSEC) -start_time3)
        print(no_error_count)
        touchled_4.set_color(Color.GREEN)
        # abs_error and last_error are used for calculating derivitive
        last_error = math.fabs(error)

        # Wating 10 milliseconds to wait for a new reading of error
        wait(10, MSEC)
        touchled_4.set_color(Color.BLUE)

        # derivative gets calculated using the last_error and the abs_error. kd will be multiplied by the derivative.
        error = target_heading - brain_inertial.rotation()
        abs_error = math.fabs(error)

        derivative = abs_error - last_error

        touchled_4.set_color(Color.YELLOW)
        # Output being calculated. Output is used to calculate which direction we need to go to make the turn at any given moment.
        output = abs_error * kp + derivative * kd + (error_total * ki)

        # This makes sure that the output never gets to slow, and that the robot will keep moving when the output is to little.
        if (output < 10):
            output = 10
        # If the error is greater than 0, then we turn the robot clockwise.
        if error > 0:
            touchled_4.set_color(Color.GREEN)
            left_motor.set_velocity(output)
            right_motor.set_velocity(output)
            left_motor.spin(FORWARD)
            right_motor.spin(REVERSE)
            # If the error is less than 0, then we turn the robot counterclockwise
        else:
            touchled_4.set_color(Color.RED)
            left_motor.set_velocity(output)
            right_motor.set_velocity(output)
            left_motor.spin(REVERSE)
            right_motor.spin(FORWARD)
            # Adding the error to get error_total, which is used for calculating output
            error = target_heading - brain_inertial.rotation()
            abs_error = math.fabs(error)
            error_total = error_total + abs_error

        if math.fabs(error) <= .75:
            no_error_count += 1
        else:
            no_error_count = 0
            #print("no_error_count: ", no_error_count)
            #print("math.fabs(error): ", math.fabs(error))

        if (brain.timer.time(MSEC) -start_time3) >= 2:
            no_error_count = 5

    # Once the while loop has been exited, meaning the turn is complete, we stop the motors
    left_motor.stop()
    right_motor.stop()

    print("End error: " + str(error))
    print("Ending heading turn: ", brain_inertial.rotation())

#Define Variables
#---------------------------------------------------------------------------------------------------------------------------------------------------------------
def high_goal(degrees):
    catapult.spin_for(REVERSE, degrees, DEGREES)

def low_goal(degree):
    catapult.spin_for(FORWARD, degree, DEGREES)

def onevent_high_max_height_0():
    # Putting catapult at highest position from lowest high goal position
    low_goal(700)

def onevent_low_high_0():
    # Loading for low_goal from high goal
    high_goal(800)

def onevent_high_low_0():
    # Loading for high goal from low_goal
    high_goal(100)

def onevent_forward_intake_0():
    # Multithread for starting intake
    intake.spin(FORWARD)
    intake_outake.spin(FORWARD)

def onevent_reverse_intake_0():
    # Multithread for stopping intake
    intake.spin(REVERSE)
    intake_outake.spin(REVERSE)
    
def onevent_stop_intake_and_outake_0():
    intake.stop()
    intake_outake.stop()

#Functions
#-------------------------------------------------------------------------------------------------------------------------------------------------


# system event handlers
low_high(onevent_low_high_0)
forward_intake(onevent_forward_intake_0)
high_max_height(onevent_high_max_height_0)
reverse_intake(onevent_reverse_intake_0)
high_low(onevent_high_low_0)
stop_intake_and_outake(onevent_stop_intake_and_outake_0)
wait(15, MSEC)

#Start Calibration
#---------------------------------------------------------------------------------------------------------------------------------------------------------------
brain_inertial.calibrate()
while brain_inertial.is_calibrating():
    sleep(50)

brain_inertial.set_heading(0, DEGREES)
# test out different brake modes, see if their is a better one
left_motor.set_stopping(BRAKE)
right_motor.set_stopping(BRAKE)
catapult.set_stopping(BRAKE)
catapult.set_velocity(100, PERCENT)
intake.set_velocity(100, PERCENT)
intake_outake.set_velocity(100,PERCENT)
#right_motor.set_timeout(4, SECONDS)
#left_motor.set_timeout(4, SECONDS)
touchled_4.set_color(Color.RED)

while not touchled_4.pressing():
    wait(5, MSEC)

touchled_4.set_color(Color.GREEN)

#PID Values in easy variables
#---------------------------------------------------------------------------------------------------------------------------------------------------------------
kp_backward = 3
kp_forward = 3.5
ki_backward = 0.004
ki_forward = 0.006
kd_backward = 0.02
kd_forward = 0.03
#---------------------------------------------------------------------------------------------------------------------------------------------------------------
#Code Start
#---------------------------------------------------------------------------------------------------------------------------------------------------------------
#---------------------------------------------------------------------------------------------------------------------------------------------------------------
degreesInMillimeter = 18.2880
forward_intake.broadcast()
start_time = brain.timer.time(MSEC)


drivestraitbackward_PID(3*degreesInMillimeter, -80, kp_backward, ki_backward, kd_backward, 0)
wait(.1,SECONDS)
drivestraitforward_PID(2*degreesInMillimeter, 80, kp_forward, ki_forward,kd_forward , 0)
wait(.2,SECONDS)
stop_intake_and_outake.broadcast()
#drivestraitbackward_PID(42*degreesInMillimeter, -80, kp_backward, ki_backward, kd_backward, 0)
drivestraitbackward_PID(36*degreesInMillimeter, -80, kp_backward, ki_backward, kd_backward, 0)
drivestraitbackward_PID(8*degreesInMillimeter, -80, kp_backward, ki_backward, kd_backward, 0)
#wait(0.,SECONDS)
high_goal(100)

forward_intake.broadcast()
wait(.85,SECONDS)
reverse_intake.broadcast()
wait(.75,SECONDS)
# s<x number of balls = 2(x+3)
s= 0
while s < 5:
    #wait(.25,SECONDS)
    forward_intake.broadcast()
    drivestraitforward_PID(29*degreesInMillimeter, 80, kp_forward, ki_forward,kd_forward , 0)
    wait(0.5,SECONDS)
    drivestraitforward_PID(2*degreesInMillimeter, 80, kp_forward, ki_forward,kd_forward , 0)
    wait(0.5,SECONDS)
    drivestraitbackward_PID(20*degreesInMillimeter, -80, kp_backward, ki_backward, kd_backward, 0)
    reverse_intake.broadcast()
    drivestraitbackward_PID(11*degreesInMillimeter, -80, kp_backward, ki_backward, kd_backward, 0)
    wait(.65,SECONDS)
    s +=1


forward_intake.broadcast()

low_high.broadcast()



drivestraitforward_PID(29*degreesInMillimeter, 80, kp_forward, ki_forward,kd_forward , 0)
wait(.5,SECONDS)
drivestraitforward_PID(2*degreesInMillimeter, 80, kp_forward, ki_forward,kd_forward , 0)
wait(.5,SECONDS)
stop_intake_and_outake.broadcast()
turn_PID(100, 0.6, 0.001, 0.001, -45)

wait(.2,SECONDS)
drivestraitbackward_PID(29*degreesInMillimeter, -80, kp_forward, ki_forward,kd_forward , -45)
wait(.2,SECONDS)
turn_PID(100, 0.6, 0.001, 0.001, 0)
wait(.2,SECONDS)
drivestraitbackward_PID(17*degreesInMillimeter, -80, kp_forward, ki_forward,kd_forward , 0)
#wait(.2,SECONDS)
high_goal(100)
wait(250, MSEC)
forward_intake.broadcast()
wait(.85,SECONDS)
reverse_intake.broadcast()
wait(.75,SECONDS)
#---------------------------------------------------------------------------------------------------------------------------------------------------------------
s= 0
while s < 4:
    #wait(.3,SECONDS)
    forward_intake.broadcast()
    drivestraitforward_PID(29*degreesInMillimeter, 80, kp_forward, ki_forward,kd_forward , 0)
    wait(0.5,SECONDS)
    drivestraitforward_PID(2*degreesInMillimeter, 80, kp_forward, ki_forward,kd_forward , 0)
    wait(0.5,SECONDS)
    drivestraitbackward_PID(20*degreesInMillimeter, -80, kp_backward, ki_backward, kd_backward, 0)
    reverse_intake.broadcast()
    drivestraitbackward_PID(11*degreesInMillimeter, -80, kp_backward, ki_backward, kd_backward, 0)
    wait(0.65,SECONDS)
    s +=1

print("Forward code ended. Time taken: " + str(brain.timer.time(MSEC) -start_time))
brain.program_stop()

