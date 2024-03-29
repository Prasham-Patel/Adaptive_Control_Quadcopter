#!/usr/bin/env python
#---------------------------------------------------
from pid import PID
import numpy as np
from feedback_control import feedback_control
import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray, Float32
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plot
#---------------------------------------------------
arr = []

def app(n, x, y, z, xd, yd, zd):
	global arr 
	arr.append([n, x, y, z, xd, yd, zd])
	return 0


def control_kwad(msg, args):
	#Declare global variables as you dont want these to die, reset to zero and then re-initiate when the function is called again.
	global roll, pitch, yaw, x, y, z, desx, desy, desz, err_roll, err_pitch, err_yaw
	global arr

	#Assign the Float64MultiArray object to 'f' as we will have to send data of motor velocities to gazebo in this format
	f = Float64MultiArray()
	
	#Convert the quaternion data to roll, pitch, yaw data
	#The model_states contains the position, orientation, velocities of all objects in gazebo. In the simulation, there are objects like: ground, Contruction_cone, quadcopter (named as 'Kwad') etc. So 'msg.pose[ind]' will access the 'Kwad' object's pose information i.e the quadcopter's pose.
	ind = msg.name.index('Kwad')
	orientationObj = msg.pose[ind].orientation
	orientationList = [orientationObj.x, orientationObj.y, orientationObj.z, orientationObj.w]
	(roll, pitch, yaw) = (euler_from_quaternion(orientationList))
	(x, y, z) = (msg.pose[ind].position.x, msg.pose[ind].position.y, msg.pose[ind].position.z)
	#print(x, y, z)
	#send roll, pitch, yaw data to PID() for attitude-stabilisation, along with 'f', to obtain 'fUpdated'
	#Alternatively, you can add your 'control-file' with other algorithms such as Reinforcement learning, and import the main function here instead of PID().
	(fUpdated, desx, desy, desz, n, a) = feedback_control(roll, pitch, yaw, x, y, z, f)
	#(fUpdated, err_roll, err_pitch, err_yaw) = PID(roll, pitch, yaw, z, f)
	
	#The object args contains the tuple of objects (velPub, err_rollPub, err_pitchPub, err_yawPub. publish the information to namespace.

	# plotx = [n][x]
	# ploty = [n][y]
	# plotz = []

	app(n, x, y, z+0.07, desx, desy, desz)
	#print(arr[-1])
	# print(len(arr))
	


	# if n < 10:
	# 	fig,crv=plot.subplots(2,2)
	# else:
	# 	crv[0,0].scatter(n,desx,color="blue",s=3)
	# 	crv[0,0].scatter(n,x,color="red",s=1.5)
	# 	crv[0,1].scatter(n,desy,color="blue",s=3)
	# 	crv[0,1].scatter(n,y,color="red",s=1.5)
	# 	crv[1,0].scatter(n,desz,color="blue",s=3)
	# 	crv[1,0].scatter(n,z,color="red",s=1.5)


	if a == True:
		plotx = plot.figure(1)
		plot.plot(np.array(arr)[:,0], np.array(arr)[:,1])
		plot.plot(np.array(arr)[:,0], np.array(arr)[:,4])
		plot.xlabel("time")
		plot.ylabel("xpos")

		ploty = plot.figure(2)
		plot.plot(np.array(arr)[:,0], np.array(arr)[:,2])
		plot.plot(np.array(arr)[:,0], np.array(arr)[:,5])
		plot.xlabel("time")
		plot.ylabel("ypos")

		plotz = plot.figure(3)
		plot.plot(np.array(arr)[:,0], np.array(arr)[:,3])
		plot.plot(np.array(arr)[:,0], np.array(arr)[:,6])
		plot.xlabel("time")
		plot.ylabel("zpos")

		plot.show()

	         

	args[0].publish(fUpdated)
	args[1].publish(desx)
	args[2].publish(desy)
	args[3].publish(desz)
	#print("Roll: ",roll*(180/3.141592653),"Pitch: ", pitch*(180/3.141592653),"Yaw: ", yaw*(180/3.141592653))
	#print(orientationObj)
	
#----------------------------------------------------

#Initiate the node that will control the gazebo model
rospy.init_node("Control")

#initiate publishers that publish errors (roll, pitch,yaw - setpoint) so that it can be plotted via rqt_plot /err_<name>  
err_rollPub = rospy.Publisher('err_roll', Float32, queue_size=1)
err_pitchPub = rospy.Publisher('err_pitch', Float32, queue_size=1)
err_yawPub = rospy.Publisher('err_yaw', Float32, queue_size=1)

#initialte publisher velPub that will publish the velocities of individual BLDC motors
velPub = rospy.Publisher('/Kwad/joint_motor_controller/command', Float64MultiArray, queue_size=4)

#Subscribe to /gazebo/model_states to obtain the pose in quaternion form
#Upon receiveing the messages, the objects msg, velPub, err_rollPub, err_pitchPub and err_yawPub are sent to "control_kwad" function.
PoseSub = rospy.Subscriber('/gazebo/model_states',ModelStates,control_kwad,(velPub, err_rollPub, err_pitchPub, err_yawPub))

rospy.spin()
