import time
import numpy as np
from math import pi, cos, sin, atan2, acos, sqrt

def trajectory(t):
	

	Waypoints = np.array([[0,0,0.5], [0.5,0.5,1]])#, [1, 1, 0.5]])#, [1, 0, 0.5], [0, 0, 0.5]])#, [1,2,1], [1,2,0]])

	s = len(Waypoints)

	v_avg = 0.04

	T = [0]*(s-1)

	i = 0
	while i < s-1:
		T[i]=(sqrt((Waypoints[i+1,0]-Waypoints[i,0])**2+(Waypoints[i+1,1]-Waypoints[i,1])**2+(Waypoints[i+1,2]-Waypoints[i,2])**2))/v_avg
		i = i+1

	i = 0
	t_index = 0

	while i < s-1:
		T2 = 0
		j = 0
		while j <=i:
			T2 = T2 + T[j]
			j=j+1
			
		if t<=T2:
			t_index = i
			break
		else:
			i = i+1
			continue

		

	
	if t_index == 0:
		t = t;
	else:
		t = t - T[t_index - 1]

	# t = min (t, T[t_index])

	
	traj = [0]*3
	traj_vel = [0]*3
	traj_acc = [0]*3

	i = 0
	while i<3:

		# if t_index == 0:
		# 	vi = 0
		# else:
		# 	vi = (Waypoints[t_index+1,i]-Waypoints[t_index,i])/(T[t_index])
		# if t_index == s-2:
		# 	vf = 0
		# else:
		# 	vf=(Waypoints[t_index+2,i]-Waypoints[t_index+1,i])/(T[t_index])
		vi = 0
		vf = 0

		Wi = Waypoints[t_index,i]
		Wf = Waypoints[t_index+1,i]

		traj[i] = ((6*Wf)/T[t_index]**5 - (6*Wi)/T[t_index]**5 - (3*vf)/T[t_index]**4 - (3*vi)/T[t_index]**4)*t**5 + ((15*Wi)/T[t_index]**4 - (15*Wf)/T[t_index]**4 + (7*vf)/T[t_index]**3 + (8*vi)/T[t_index]**3)*t**4 + ((10*Wf)/T[t_index]**3 - (10*Wi)/T[t_index]**3 - (4*vf)/T[t_index]**2 - (6*vi)/T[t_index]**2)*t**3 + vi*t + Wi

		traj_vel[i] = ((30*Wf)/T[t_index]**5 - (30*Wi)/T[t_index]**5 - (15*vf)/T[t_index]**4 - (15*vi)/T[t_index]**4)*t**4 + ((60*Wi)/T[t_index]**4 - (60*Wf)/T[t_index]**4 + (28*vf)/T[t_index]**3 + (32*vi)/T[t_index]**3)*t**3 + ((30*Wf)/T[t_index]**3 - (30*Wi)/T[t_index]**3 - (12*vf)/T[t_index]**2 - (18*vi)/T[t_index]**2)*t**2 + vi

		traj_acc[i] = ((120*Wf)/T[t_index]**5 - (120*Wi)/T[t_index]**5 - (60*vf)/T[t_index]**4 - (60*vi)/T[t_index]**4)*t**3 + ((180*Wi)/T[t_index]**4 - (180*Wf)/T[t_index]**4 + (84*vf)/T[t_index]**3 + (96*vi)/T[t_index]**3)*t**2 + ((60*Wf)/T[t_index]**3 - (60*Wi)/T[t_index]**3 - (24*vf)/T[t_index]**2 - (36*vi)/T[t_index]**2)*t

		i = i+1


	
	return traj[0], traj[1], traj[2], traj_vel[0], traj_vel[1], traj_vel[2], traj_acc[0], traj_acc[1], traj_acc[2], T[s-2]

