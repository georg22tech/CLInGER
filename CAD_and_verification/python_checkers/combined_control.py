#import libraries for IMU, motor interfacing, maths

########## SETUP ##########
###set pin values for constants
#input pins (from HRI):
PIN_JOYSTICK_1 = 0
PIN_JOYSTICK_2 = 0
#control pins for inching motor 1

#instantiate variables

########## FUNCTION DEFINITIONS ##########
#function to set up a motor

#function to update direction

#function to update intended position

#function to move continuum with control (PID and kinematics)

#function to move an inching unit with control (PID and slipping checks)

#function to check for slipping

#function for error calculation, returning Euclidian and axial errors

########## MAIN ##########
#set up motors, defining each section of robot

#repeate until keyboard interrupt

#update current position

#update directions

#set intended position

#check continuum and correct

#check slipping and correct

#calculate position errors

#move continuum

#move inching unit