#-------------
import time
import numpy as np
from trajectory import trajectory
from math import pi, cos, sin, atan2, acos, sqrt
#--------------------------------------------------------------------------------------------------------------------------------------
def feedback_control(roll, pitch, yaw, x, y, z, f):
	#Define the global variables to prevent them from dying and resetting to zero, each time a function call occurs. Some of these variables 		may be redundant.
	global n, a, kp_x, kd_x, ki_x, kp_y, kd_y, ki_y, kp_z, kd_z, ki_z, kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, kp_yaw, ki_yaw, kd_yaw, x_des, y_des, z_des, dx_des, dy_des, dz_des, ddx_des, ddy_des, ddz_des, yaw_des, x_prev, y_prev, z_prev, roll_prev, pitch_prev, yaw_prev, prevTime, T, sampleTime, flag, g, m
	#prevErr_roll, prevErr_pitch, prevErr_yaw, pMem_roll, pMem_yaw, pMem_pitch, iMem_roll, iMem_pitch, iMem_yaw, dMem_roll, dMem_pitch, dMem_yaw, flag, setpoint, sampleTime

	#-----------------------
	#Assign your PID values here. From symmetry, control for roll and pitch is the same.
	kp_x = 0.3
	kd_x = 3.9
	ki_x = 0
	kp_y = kp_x
	kd_y = kd_x
	ki_y = 10
	kp_z = 10
	kd_z = 5
	ki_z = 8
	kp_roll = 170#150#120
	ki_roll = 0.0002
	kd_roll = 330#250
	kp_pitch = kp_roll
	ki_pitch = ki_roll
	kd_pitch = kd_roll
	kp_yaw = 10
	ki_yaw = 0
	kd_yaw = 5
	flag = 0
	sampleTime = 0
	# Defining dynamic parameters
	g = 9.81
	m = 1
	r = 0
	Ipitch = 0.0451
	Iroll = 0.0451
	Iyaw = 0.0170
	#-----------------------



	# Defining destination variables from Trajectory
	# x_des = 0
	# y_des = 0
	# z_des = 0
	# dx_des = 0
	# dy_des = 0
	# dz_des = 0
	# ddx_des = 0
	# ddy_des = 0
	# ddz_des = 0
	yaw_des = 0

	

	#Reset the following variables during the first run only.
	if z <= 0.05:
		prevTime = 0
		x_prev = 0
		y_prev = 0
		z_prev = 0
		roll_prev = 0
		pitch_prev = 0
		yaw_prev = 0
		n = 0
		T = time.time()
	#------------------------
	#Define dt, dy(t) here for kd calculations.

	currTime = time.time() - T
	t = currTime

	# Defining destination variables from Trajectory
	#print("in")
	(x_des, y_des, z_des, dx_des, dy_des ,dz_des, ddx_des, ddy_des, ddz_des, final_t) = trajectory(currTime)
	# x_des = (4103230296004839*t**5)/2361183241434822606848 - (6*t**4)/78125 + (8348043408212971*t**3)/9223372036854775808
	# dx_des = (5129037870006049*t**4)/590295810358705651712 - (24*t**3)/78125 + (391314534759983*t**2)/144115188075855872
	# ddx_des = (5129037870006049*t**3)/147573952589676412928 - (72*t**2)/78125 + (391314534759983*t)/72057594037927936
	# y_des = x_des
	# dy_des = dx_des
	# ddy_des = ddx_des
	# z_des = 0.5
	# dz_des = 0
	# ddz_des = 0
	# print ("---------")
	# print (x_des,y_des, z_des)
	
	dt = (currTime - prevTime)
	
	#-------------------------------------------------------------------------------------------------------------------------------
	
	if(abs(dt) >= 0.0005):
		dx = (x - x_prev)/dt
		dy = (y - y_prev)/dt
		dz = (z - z_prev)/dt
		droll = (roll - roll_prev)/dt
		dpitch = (pitch - pitch_prev)/dt
		dyaw = (yaw - yaw_prev)/dt
	

	else:
		dx = 0
		dy = 0
		dz = 0
		droll = 0
		dpitch = 0
		dyaw = 0
		
	#Store the current variables into previous variables for the next iteration.
	prevTime = currTime
	x_prev = x
	y_prev = y
	z_prev = z
	roll_prev = roll
	pitch_prev = pitch
	yaw_prev = yaw

	#Defining Control law
	Rx = ddx_des + kd_x*(dx_des - dx) + kp_x*(x_des - x)
	Ry = ddy_des + kd_y*(dy_des - dy) + kp_y*(y_des - y)
	Rz = ddz_des + kd_z*(dz_des - dz) + kp_z*(z_des - z)

	roll_des = atan2((Rx*sin(yaw_des) - Ry*cos(yaw_des)), g)
	pitch_des = atan2((Rx*cos(yaw_des) + Ry*sin(yaw_des)), g)


	# if roll_des >= 0.1: roll_des = 0.1
	# if roll_des <= -0.1: roll_des = -0.1
	# if pitch_des >= 0.1: pitch_des = 0.1
	# if pitch_des <= -0.1: pitch_des = -0.1

	Mpitch = kp_pitch*(pitch_des - pitch) + kd_pitch*(0 - dpitch) #M1
	Mroll = kp_roll*(roll_des - roll) + kd_roll*(0 - droll)       #M2
	Myaw = kp_yaw*(yaw_des - yaw) + kd_yaw*(0 - dyaw)             #M3

	U1 = (Rz + g)*m/(cos(pitch)*cos(roll))

	U2 = (Ipitch*Mpitch - g*m*r*sin(pitch))/0.16

	U3 = (Iroll*Mroll - g*m*r*sin(roll))/0.16

	U4 = Iyaw*Myaw


	Tbr = (U1 + U2 - U3 - U4)

	Tfr = (U1 - U2 - U3 + U4)

	Tbl = (U1 + U2 + U3 + U4)

	Tfl = (U1 - U2 + U3 - U4)


	c = 1

	esc_br = c*970*Tbr/(m*g)
	esc_fr = c*970*Tfr/(m*g)
	esc_bl = c*970*Tbl/(m*g)
	esc_fl = c*970*Tfl/(m*g)

	
	#print(esc_br, esc_fr, esc_bl, esc_fl)
	#print(Tbr, Tfr, Tbl, Tfl)
	# print (x, y, z)

	err_pitch = float(pitch - pitch_des)*(180 / 3.141592653) 
	err_roll = float(roll - roll_des)*(180 / 3.141592653)
	err_yaw = float(yaw - yaw_des)*(180/3.14159263)

	#-------------------------------------------------------------------------------------------------------------------------------
	
	#Limit the ESC pulses to upper limit and lower limit, in case the PID algorithm goes crazy and high af.
	if(esc_br > 2400): esc_br = 2400
	if(esc_bl > 2400): esc_bl = 2400
	if(esc_fr > 2400): esc_fr = 2400
	if(esc_fl > 2400): esc_fl = 2400
	
	if(esc_br < 10): esc_br = 10
	if(esc_bl < 10): esc_bl = 10
	if(esc_fr < 10): esc_fr = 10
	if(esc_fl < 10): esc_fl = 10
	a = False

	# print(final_t)
	if currTime >= 18:
	 	esc_br = 0#1500*Tbr/(m*g)
	 	esc_fr = 0#1500*Tfr/(m*g)
	 	esc_bl = 0#1500*Tbl/(m*g)
	 	esc_fl = 0#1500*Tfl/(m*g)
	 	a = True
	 	print ("done")


	
	#Map the esc values to motor values
	br_motor_vel = ((esc_br - 1500)/25) + 50
	bl_motor_vel = ((esc_bl - 1500)/25) + 50
	fr_motor_vel = ((esc_fr - 1500)/25) + 50
	fl_motor_vel = ((esc_fl - 1500)/25) + 50
	#----------------------------------------------------------------------------------------------------------------------------------

	f.data = [fl_motor_vel,-bl_motor_vel,br_motor_vel, -fr_motor_vel]
	
	#Return these variables back to the control file.
	return f, x_des, y_des, z_des, currTime, a
	n = n + 1
#--------------------------------------------------------------------------------------------------------------------------------------
