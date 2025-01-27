#region VEXcode Generated Robot Configuration
from vex import *
import urandom

# Brain should be defined by default
brain=Brain()

# Robot configuration code
brain_inertial = Inertial()
left_motor = Motor(Ports.PORT1, False)
right_motor = Motor(Ports.PORT7, True)
catapult_motor_a = Motor(Ports.PORT6, False)
catapult_motor_b = Motor(Ports.PORT12, True)
catapult = MotorGroup(catapult_motor_a, catapult_motor_b)
intake_motor_a = Motor(Ports.PORT5, True)
intake_motor_b = Motor(Ports.PORT11, False)
intake = MotorGroup(intake_motor_a, intake_motor_b)
touchled_10 = Touchled(Ports.PORT10)
distance_3 = Distance(Ports.PORT3)



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


        

# register for intake and catapult events
forward_intake = Event()
reverse_intake = Event()
low_high = Event()
high_max_height = Event()
high_low = Event()

# code for making left and right motors as drive train so they move together
def move(left_speed, right_speed):
   left_motor.set_velocity(left_speed, PERCENT)
   right_motor.set_velocity(right_speed, PERCENT)
   left_motor.spin(FORWARD)
   right_motor.spin(FORWARD)

def drivestraitforward_PID(target_distance:int, speed:int, kp:float, ki:float, kd:float, target_heading:int):   
   #brain_inertial.set_rotation(0, DEGREES)
   left_motor.set_position(0, DEGREES)
   right_motor.set_position(0, DEGREES)

   #print("Starting heading forward: ", brain_inertial.rotation())

   # based on the current heading find the change in heading needed to achieve target heading
   startdistance = distance_3.object_distance(MM)   
   distance_moved = distance_3.object_distance(MM) - startdistance

   # error_total is used for K(I)
   error_total = brain_inertial.rotation() - target_heading
   #print("Start Error total: ", error_total)
    
   while (math.fabs(left_motor.position(DEGREES)) + math.fabs(right_motor.position(DEGREES))) / 2 < target_distance and distance_moved < target_distance:       
       distance_moved = distance_3.object_distance(MM) - startdistance
       #print("Distance moved forward:", distance_moved)
       #print("Distance moved forward:", distance_moved)
       error = brain_inertial.rotation() - target_heading
       last_error = error
       wait(1, MSEC)
       error = brain_inertial.rotation() - target_heading       
       # error_total is used for K(D)
       derivative = error - last_error

       # caculate velocity offset based on k(P), K(I) and K(D)
       output = error * kp + derivative * kd + (error_total * ki)
       #print("Output Forward: ", output)

       move(speed - output, speed + output)
       error_total = error_total + error

   left_motor.stop()
   right_motor.stop()
   #print("Ending heading forward: ", brain_inertial.rotation())
   #print("End Error total: ", error_total)
   #print("Distance moved: ", distance_moved)   

def drivestraitbackward_PID(target_distance:int, speed:int, kp:float, ki:float, kd:float, target_heading:int):   
   #brain_inertial.set_rotation(0, DEGREES)
   left_motor.set_position(0, DEGREES)
   right_motor.set_position(0, DEGREES)
   #print("Starting heading backward: ", brain_inertial.rotation())

   # based on the current heading find the change in heading needed to achieve target heading
   startdistance = distance_3.object_distance(MM)   
   distance_moved = math.fabs(distance_3.object_distance(MM) - startdistance)

   # error_total is used for K(I)
   error_total = brain_inertial.rotation() - target_heading
   #print("Start Error total: ", error_total)
   startdistance = distance_3.object_distance(MM)   
   distance_moved =  startdistance - distance_3.object_distance(MM)
   print("Start distance for backward ",startdistance)

   while (distance_3.object_distance(MM) > 15 and (math.fabs(left_motor.position(DEGREES)) + math.fabs(right_motor.position(DEGREES))) / 2 < target_distance): #or distance_moved < target_distance:
       distance_moved = startdistance - distance_3.object_distance(MM)
       #print("Distance moved backward: ", distance_moved)
    
       #print("Distance from goal: ", distance_3.object_distance(MM))

       error = brain_inertial.rotation() - target_heading
       last_error = error
       wait(1, MSEC)
       error = brain_inertial.rotation() - target_heading
       # error_total is used for K(D)
       derivative = error - last_error

       # caculate velocity offset based on k(P), K(I) and K(D)
       output = error * kp + derivative * kd + (error_total * ki)

       #print("Output Backwards: ", output)
       move(speed - output, speed + output)
       error_total = error_total + error

   move(speed, speed)
   sleep(500)
   left_motor.stop()
   right_motor.stop()
   print("Ending heading backward: ", brain_inertial.rotation())
   #print("End Error total: ", error_total)
   #print("Distance moved: ", distance_moved)   

'''
We take in the parameters of speed for how fast we want to go.
kp, ki, and kd are values that affect the output (how it turns)
The target heading tells us where we want to go.
kp is the rate of change that the robot moves.
ki takes the past error adds it up, and is used to change the rate of change
kd uses that past to predicte future outcomes, and changes the rate of change
We  use print statements and led for debuging all throughout the code. This helps us tune the code to work.
'''
def turn_PID(speed:float, kp:float, ki:float, kd:float, target_heading:int):
   # Setting the rotation and position to 0 as a starting point
   # Setting the motors to brake so that they don't drift. 
   #brain_inertial.set_rotation(0, DEGREES)   
   left_motor.set_position(0, DEGREES)
   right_motor.set_position(0, DEGREES)
   print("Starting heading turn: ", brain_inertial.rotation())

   # Calculating error total for ki
   error_total = target_heading - brain_inertial.rotation()
   # Print statements and led for debuging
  # print("error_total", error_total)
   touchled_10.set_color(Color.RED)

   # Setting the error for a starting value.
   error = target_heading - brain_inertial.rotation()
   derivative = 0
   
   no_error_count = 0
   # Loop that moves robot until there is no error
   while math.fabs(error) > 0.75 or no_error_count <= 3:       
       touchled_10.set_color(Color.GREEN)
       # abs_error and last_error are used for calculating derivitive
       last_error = math.fabs(error)
       #print("last_error", last_error)

       # Wating 10 milliseconds to wait for a new reading of error
       wait(5, MSEC)
       touchled_10.set_color(Color.BLUE)

       # derivative gets calculated using the last_error and the abs_error. kd will be multiplied by the derivative.
       error = target_heading - brain_inertial.rotation()       
       abs_error = math.fabs(error)
       #print("abs_error", abs_error)

       derivative = abs_error - last_error

       touchled_10.set_color(Color.YELLOW)
       # Output being calculated. Output is used to calculate which direction we need to go to make the turn at any given moment.
       output = abs_error * kp + derivative * kd + (error_total * ki)
       #print("output", output)

       # This makes sure that the output never gets to slow, and that the robot will keep moving when the output is to little.
       if (output < 5):
           output = 5
       
       # If the error is greater than 0, then we turn the robot clockwise.
       if error > 0:  
           #print("Positive")
           touchled_10.set_color(Color.GREEN)  
           left_motor.set_velocity(output)
           right_motor.set_velocity(output)
           left_motor.spin(FORWARD)
           right_motor.spin(REVERSE)
       # If the error is less than 0, then we turn the robot counterclockwise
       else:
           #print("Negative")  
           touchled_10.set_color(Color.RED)
           left_motor.set_velocity(output)
           right_motor.set_velocity(output)
           left_motor.spin(REVERSE)
           right_motor.spin(FORWARD)
       # Adding the error to get error_total, which is used for calculating output
       error = target_heading - brain_inertial.rotation()
       abs_error = math.fabs(error)
       error_total = error_total + abs_error

       if math.fabs(error) <= 0.75:
           no_error_count += 1
       else:
           no_error_count = 0
       print("no_error_count: ", no_error_count)
       print("math.fabs(error): ", math.fabs(error))
   
   # Once the while loop has been exited, meaning the turn is complete, we stop the motors
   left_motor.stop()
   right_motor.stop()

   print("End error: " + str(error))
   print("Ending heading turn: ", brain_inertial.rotation())
#--------------------------------------------------------------------------------
def high_goal(degrees):
   # High goal 100
   catapult.spin_for(REVERSE, degrees, DEGREES)

def low_goal(degree):
   # low_goal 300
   catapult.spin_for(FORWARD, degree, DEGREES)

def onevent_high_max_height_0():
   # Putting catapult at highest position from lowest high goal position
   low_goal(700)


def onevent_low_high_0():
   # Loading for low_goal from high goal
   high_goal(500)


def onevent_high_low_0():
   # Loading for high goal from low_goal
   high_goal(1000)

def onevent_forward_intake_0():
   # Multithread for starting intake
   intake.spin(FORWARD)

def onevent_reverse_intake_0():
   # Multithread for stopping intake
   intake.spin(REVERSE)
#---------------------------------------------------------------------

def clear_switch():
    intake.stop()
    high_max_height.broadcast()
    #drivestraitbackward_PID(980, -100, kp_backward, ki_backward, kd_backward, -90)
    
    #low_goal(700)
    wait(.3, SECONDS)
    forward_intake.broadcast()
    wait(1.3, SECONDS)
    intake.stop()
    reverse_intake.broadcast()
    wait(0.1,MSEC)
    intake.stop()
    high_low.broadcast()
    
    #forward_intake.broadcast()
    wait(1, SECONDS)
    high_goal(100)
    #forward_intake.broadcast()
    wait(1, SECONDS)
    high_low.broadcast()
    wait(1, SECONDS)
    #forward_intake.broadcast()







def moveandscore():
   #print("heading start: ", brain_inertial.rotation())
   #drivestraitbackward_PID(980, -62.5, kp_backward, ki_backward, kd_backward, -90) #0.456, 0.05664596, 0.45885, 0)
   #wait(500,MSEC)
   # Shoot High
   #high_goal(100)
   #Loading for Low
   #low_high.broadcast()

#low_goal(500)
#wait(1,SECONDS)
#onevent_high_low_0()
#wait(1,SECONDS)
#high_goal(100)

   x=1
   while x <= 4:  
       #wait(3, SECONDS)
       #high_low.broadcast()
       forward_intake.broadcast()
       drivestraitforward_PID(635, 100, kp_forward, ki_forward, kd_forward, -90)
       wait(2, SECONDS)
       intake.stop()
       drivestraitbackward_PID(635, -100, kp_backward, ki_backward, kd_backward, -90) #0.456, 0.05664596, 0.45885, 0)
       forward_intake.broadcast()
       wait(2.5, SECONDS)
       #low_goal(500)
       x += 1
       if x > 4:
            forward_intake.broadcast()
            drivestraitforward_PID(635, 100, kp_forward, ki_forward, kd_forward, -90)
            wait(2, SECONDS)
            intake.stop()
            drivestraitbackward_PID(635, -100, kp_backward, ki_backward, kd_backward, -90) #0.456, 0.05664596, 0.45885, 0)
            forward_intake.broadcast()
            
       #low_high.broadcast()
       #else:
        #high_low.broadcast()
        #drivestraitforward_PID(635, 62.5, kp_forward, ki_forward, kd_forward, -90)
        #print("heading end: ", brain_inertial.rotation())
        

# system event handlers
low_high(onevent_low_high_0)
forward_intake(onevent_forward_intake_0)
high_max_height(onevent_high_max_height_0)
reverse_intake(onevent_reverse_intake_0)
high_low(onevent_high_low_0)
# add 15ms delay to make sure events are registered correctly.
wait(15, MSEC)




#when_started1()
# We calculate how long it takes for the turn to be completed
#time1 = brain.timer.time()
# We have tunned kp, ki, and kd to make accurate turns.
# This code makes a 90 degree turn
#turn_PID(100, 0.75, 0.0001, 0.0001, 90)
#time2 = brain.timer.time()
#print("time: " + str(time2 -time1))
brain_inertial.calibrate()
while brain_inertial.is_calibrating():
   sleep(50)
brain_inertial.set_rotation(0, DEGREES)
left_motor.set_stopping(BRAKE)
right_motor.set_stopping(BRAKE)
catapult.set_stopping(BRAKE)
catapult.set_velocity(100, PERCENT)
intake.set_velocity(100, PERCENT)
right_motor.set_timeout(4, SECONDS)
left_motor.set_timeout(4, SECONDS)
touchled_10.set_color(Color.RED)
catapult.set_timeout(1, SECONDS)
while not touchled_10.pressing():
        wait(20, MSEC)
touchled_10.set_color(Color.GREEN)
intake.spin(FORWARD)
kp_backward = 0
kp_forward = 0
ki_backward = 0
ki_forward = 0
kd_backward = 0
kd_forward = 0

forward_intake.broadcast()

start_time = brain.timer.time(MSEC)
drivestraitforward_PID(230, 100, kp_forward, ki_forward,kd_forward , 0)
wait(125,MSEC)
intake.stop()
print("Forward code ended. Time taken: " + str(brain.timer.time(MSEC) -start_time))

time1 = brain.timer.time(MSEC)
turn_PID(100, 0, 0, 0, -90)
wait(125,MSEC)
print("Turn code ended. Time taken: " + str(brain.timer.time(MSEC) -time1))

time1 = brain.timer.time(MSEC)
clear_switch()
print("clear switch code ended. Time taken: " + str(brain.timer.time(MSEC) -time1))
wait(125,MSEC)
time1 = brain.timer.time(MSEC)
forward_intake.broadcast()
drivestraitforward_PID(635, 100, kp_forward, ki_forward, kd_forward, -90)
print("Move code ended. Time taken: " + str(brain.timer.time(MSEC) -time1))
forward_intake.broadcast()
wait(2,SECONDS)
time1 = brain.timer.time(MSEC)
intake.stop()
turn_PID(100, 0, 0, 0, -155)
print("Turn code 2 ended. Time taken: " + str(brain.timer.time(MSEC) -time1))
intake.stop()
wait(125,MSEC)
#forward_intake.broadcast()
#high_low.broadcast()
time1 = brain.timer.time(MSEC)
intake.stop()
drivestraitbackward_PID(275, -11, kp_backward, ki_backward, kd_backward, -155)
print("Backward Transition 1 ended. Time taken: " + str(brain.timer.time(MSEC) -time1))
wait(1,SECONDS)

time1 = brain.timer.time(MSEC)
turn_PID(100, 0, 0, 0, -90)
print("Total time taken: " + str(brain.timer.time(MSEC) -start_time))
print("Turn code 3 ended. Time taken: " + str(brain.timer.time(MSEC) -time1))
wait(0.5,SECONDS)

#time1 = brain.timer.time(MSEC)
#drivestraitforward_PID(318.5, 62.5, kp_forward, ki_forward,kd_forward , -90)
#print("Pick up ball. Time taken: " + str(brain.timer.time(MSEC) -time1))
#wait(500,MSEC)

time1 = brain.timer.time(MSEC)
drivestraitbackward_PID(300, -100, kp_backward, ki_backward, kd_backward, -90)
print("Shoot ball. Time taken: " + str(brain.timer.time(MSEC) -time1))
wait(250,MSEC)


time1 = brain.timer.time(MSEC)
#drivestraitbackward_PID(784, -62.5, kp_backward, ki_backward, kd_backward, -90)
clear_switch()
print("Shoot ball. Time taken: " + str(brain.timer.time(MSEC) -time1))
wait(125,MSEC)
forward_intake.broadcast()


time1 = brain.timer.time(MSEC)
moveandscore()
print("Trials 2 ended. Time taken: " + str(brain.timer.time(MSEC) -time1))
brain.program_stop()


print("Path ended. Time taken: " + str(brain.timer.time(MSEC) - start_time))



