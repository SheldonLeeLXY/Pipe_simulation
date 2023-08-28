#!/usr/bin/env python

import rospy
import message_filters
import math
import time
import os
import os.path
import rospkg

from sensor_msgs.msg import Imu
from message_filters import TimeSynchronizer, Subscriber
from std_msgs.msg import Float64
from math import sin,cos,atan2,sqrt,fabs,pi
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import Range
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
import PID 

# pidl = PID.PID(0.2, 0.15, 0.5)
# pidr = PID.PID(0.2, 0.15, 0.5)
# pidl.setSampleTime(0.01)
# pidr.setSampleTime(0.01)

base_speed = 3
CONTROL_LIMIT = 6
controlrate = 100 #low level control rate in miliseconds 
sensorrate = 10 #in Hz == rospy spin rate

flcontrol = base_speed
frcontrol = base_speed
blcontrol = base_speed
brcontrol = base_speed
# commands, filter constants
fweigh = [2, 4, 6, 8, 9, 10, 11, 12, 13, 25]

count = 0
# global variables
timestep = 0
dis_front = [0]*10
dis_left = [0]*10
dis_right = [0]*10
imu_roll = [0]*10
imu_pitch = [0]*10
# imu_yaw = [0]*10
currentJointState = JointState()
measureON = False
INTERVALLIMIT = 200
measureINTERVAL = INTERVALLIMIT

measurePROGRESS = 0
campath = [0]*72
ind = 0
states = [0]*10  # state estimated based on laser distance sensors
hlcontrol = 0
hrcontrol = 0
camcontrol = 0
sum_hlcontrol = 0
sum_hrcontrol = 0
# constants for operating in with:
# pipe 150mm(inner diameter)
# robot wheels' diameter 56mm
PIPE_DIA = 200
WHEEL_DIA = 28
WHEEL_DIS = 42
ENCODER_RESOLUTION = 1250  # number of encoder counts when wheel turns 1 round
# CRASH_FRONT --> DE_FRONT --> WALL_FRONT --> MH_FRONT
CRASH_FRONT = 40
DE_FRONT = 65#50#45#65#75
WALL_FRONT = PIPE_DIA
MH_FRONT = 400  # 500

DE_SIDE = DE_FRONT*1.41
WALL_SIDE = WALL_FRONT*1.41
MH_SIDE = MH_FRONT*1.41 #45deg sensors setup

SIDEDIFF = 20 #15 for 150mm pipe  #25
SIDEDIFF_MAX = 80#63 for 150mm pipe
STEERATCROSS = 220
SIDE_PIPE = 180 #180	#200 #120 # 3X(75X1.41) larger than pipe diameter <-->has side pipe
SIDE_TURN2 = 125
# robot should take new commands if it has done 75% movement of last command
MOVEDONE_R = 0.75
# or take new commands when time limit has reached 90% (10% is counted for USB transmit delay)
TIMEDONE_R = 0.9
#Define a whegbot joint positions publisher for joint controllers.

def mybot_joint_velocity_publisher():

	#Initiate node for controlling joints
	rospy.init_node('mybot_joint_velocity_node', anonymous=True)

	#Define publishers for each joint position controller commands.
	pub1 = rospy.Publisher('/mybot/joint1_velocity_controller/command', Float64, queue_size=10)
	pub2 = rospy.Publisher('/mybot/joint2_velocity_controller/command', Float64, queue_size=10)
	pub3 = rospy.Publisher('/mybot/joint3_velocity_controller/command', Float64, queue_size=10)
	pub4 = rospy.Publisher('/mybot/joint4_velocity_controller/command', Float64, queue_size=10)
	pub5 = rospy.Publisher('/mybot/joint5_velocity_controller/command', Float64, queue_size=10)
	pub6 = rospy.Publisher('/mybot/joint6_velocity_controller/command', Float64, queue_size=10)
	pub7 = rospy.Publisher('/mybot/joint7_position_controller/command', Float64, queue_size=10)
	rate = rospy.Rate(10) #10 Hz

	#While loop to have joints follow a certain position, while rospy is not shutdown.
	i = 0
	while not rospy.is_shutdown():
		if i<10:
			pub1.publish(0)
			pub2.publish(0)
			pub3.publish(0)
			pub4.publish(0)
			pub5.publish(0)
			pub6.publish(0)
			pub7.publish(0)
		else:
			gazebo_sensors()

		i = i+1 #increment i

		rate.sleep() #sleep for rest of rospy.Rate(100)

def jointStatesCallback(msg):
  global currentJointState
  currentJointState = msg
#   print currentJointState.header
#   print currentJointState.position
def gazebo_sensors():
	# Setup subscriber to atlas states
	global currentJointState
  	# mybotJointState = message_filters.Subscriber('/mybot/joint_states', JointState)
	rospy.Subscriber("/mybot/joint_states", JointState, jointStatesCallback)
	ir_front = message_filters.Subscriber('/sensor/ir_front', Range)
	ir_left = message_filters.Subscriber('/sensor/ir_left', Range)
	ir_right = message_filters.Subscriber('/sensor/ir_right', Range)
	imudata = message_filters.Subscriber('/imu', Imu)
	ts = message_filters.ApproximateTimeSynchronizer([ir_front, ir_left, ir_right, imudata], 10, 0.1, allow_headerless=True)
	ts.registerCallback(process_data)
	rospy.spin()

# def process_data(ir_front,ir_left, ir_right,imudata, mybotJointState):
def process_data(ir_front,ir_left, ir_right,imudata):
	global ind, dis_front, dis_left, dis_right, imu_roll, imu_pitch, imu_yaw
	global hlcontrol, hrcontrol
	global timestep
	timestep = timestep + 1
	IMUquaternion = imudata.orientation
	qvalue = [IMUquaternion.x,IMUquaternion.y,IMUquaternion.z,IMUquaternion.w] #creates list from quaternion since it was not originally
	imu_rpy = euler_from_quaternion(qvalue) #transfrom from quaternion to euler angles

	temp_front = 1000*ir_front.range
	temp_left = 1000*ir_left.range
	temp_right = 1000*ir_right.range
	temp_roll = imu_rpy[0]#as simulation setup start position follow y, roll->x; pitch->y;
	temp_pitch = imu_rpy[1]# follow this https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
	# temp_yaw = imu_rpy[2]

	dis_front.append(temp_front)
	dis_left.append(temp_left)
	dis_right.append(temp_right)
	imu_roll.append(temp_roll)
	imu_pitch.append(temp_pitch)
	# imu_yaw.append(temp_yaw)
	del dis_front[0]
	del dis_left[0]
	del dis_right[0]
	del imu_roll[0]
	del imu_pitch[0]
	# del imu_yaw[0]

	ind = ind + 1
	if ind < 1: # wait 2 cycles ~ 2*100ms to process high level control, so each state is for 1second
		lowcontrol(hlcontrol, hrcontrol, temp_front, temp_left, temp_right, imu_rpy[0], imu_rpy[1])
	else:
		ind = 0
		front = 0
		left = 0
		right = 0
		roll = 0
		pitch = 0
		# yaw = 0
		for i in range(len(fweigh)):
			front = front + dis_front[i]*fweigh[i]/100
			left = left + dis_left[i]*fweigh[i]/100
			right = right + dis_right[i]*fweigh[i]/100
			roll = roll + imu_roll[i]*fweigh[i]/100
			pitch = pitch + imu_pitch[i]*fweigh[i]/100
			# yaw = yaw + imu_yaw[i]*fweigh[i]/100
		state_estimation(front, left, right, roll, pitch)
		highcontrol(front, left, right, roll, pitch)
		
def state_estimation(front, left, right, roll, pitch):#this func estimate robot state and decide
	global states
	global measureON
	global measurePROGRESS
	global measureINTERVAL
	state = 0
	print('front:', front)
	print('left:', left)
	print('right:', right)
	print('roll:', roll)
	print('pitch:', pitch)
	if (measureON == False) and (states[9] == states[8]) and (states[8] == states[7]):
		if measureINTERVAL >= INTERVALLIMIT:
			if states[9] == 2 or states[9] == 3 or states[9] == 5 or states[9] == 6 or states[9] == 7 or states[9] == 9:
				measureON = True
				state = states[9]
				measurePROGRESS = 0
				measureINTERVAL = 0

	if (measureON == False):
		if front >= MH_FRONT and left >= MH_SIDE and right >= MH_SIDE:  # open space <-> manhole detected
			state = 9  
		elif front <= CRASH_FRONT:
			if fabs(roll) < 0.5236:#only when robot is on even surface
				state = 8 #crash state, to turn the robot around
			else:
				state = 11 #inherit state 1 in straight pipe but deal slower

		elif (front <= DE_FRONT and left <= DE_SIDE and right <= DE_SIDE) or (front+left+right)<180:
				if fabs(roll) < 0.5236:#only when robot is on even surface
					state = 4  # Dead-End
				else:
					state = 11 #roll too much upward

		elif front > PIPE_DIA*1.5:
			if right > SIDE_PIPE:# and left < SIDE_TURN2:
				state = 3  # Right-Branch
			elif left > SIDE_PIPE and right < SIDE_TURN2: 
				state = 2  # Left-Branch-->go straight
			elif left < SIDE_PIPE and right < SIDE_PIPE:
				state = 1 # ROBOT IS IN STRAIGH PIPE
				if fabs(right - left) < SIDEDIFF:
					state = 0  # at center of pipe

		elif front <= PIPE_DIA*1.4 and front > CRASH_FRONT:#only conclude on T-junction or corner when near front wall
			if (right > SIDE_TURN2 and left > SIDE_TURN2 and states[9] == 7) or (right > SIDE_PIPE and left > SIDE_PIPE):
					state = 7  # T-Junction
					if states[9] == 7 and states[0] == 7 and fabs(roll) < 0.5236:
						state = 8 # crash state: to prevent stuck at pipe edge

			elif (right > SIDE_TURN2 and left < SIDE_TURN2 and states[9] == 6) or (right > SIDE_PIPE and left < SIDE_PIPE):
					state = 6  # Right-Turn
			elif (right < SIDE_TURN2 and left > SIDE_TURN2 and states[9] == 5) or (right < SIDE_PIPE and left > SIDE_PIPE):
				if states[9] == 7 and states[8] == 7:
					state = 7
				else:
					state = 5  # Left-Turn
					# state = 1
		elif (fabs(roll)>0.2 and front<80 and (front+left+right)<210):
			state = 4#10	
		elif front > CRASH_FRONT: #DE_FRONT:# include: 1.1*PIPE_DIA < front < 1.5*PIPE_DIA and all other case
			if left < SIDE_TURN2 or right < SIDE_TURN2:
				state = 1#sided to one side of pipe-->keep speed, steer back
			else: 
				state = 0#at centerline-->accelerate
		else:
			state = 10 # stuck or blinded --> need self adjusting to escape the situation, do not go forward
			# if none of these case, don't update state-->Old state remains

	states.append(state)
	del states[0]
	print ('State-------------------------: ', states[9])
	
def highcontrol(front, left, right, roll, pitch):
	global hlcontrol, hrcontrol, camcontrol
	global sum_hlcontrol, sum_hrcontrol
	global measureON
	global measurePROGRESS
	global measureINTERVAL
	global currentJointState
	global campath
	# start of high level control algorithm
	if measureON == True:
		# stop robot, rotate camera or carry out inspection task
		if measurePROGRESS == 0:
			# calculate path/plan for measurement
			# here we do simple rotating of camera
			startpath = currentJointState.position[6]
			endpath = currentJointState.position[6] + 2*pi
			for i in range(len(campath)):
				campath[i] = (i+1)*((endpath - startpath)/len(campath)) + startpath
		
		camcontrol = campath[measurePROGRESS]
		measurePROGRESS = measurePROGRESS + 1
		hlcontrol = 0
		hrcontrol = 0
		if measurePROGRESS >= len(campath):
			measurePROGRESS = 0
			measureON = False
			measureINTERVAL = 0

	else:
		measureINTERVAL = measureINTERVAL + 1
		camcontrol = currentJointState.position[len(currentJointState.position)-1]
		print(camcontrol)
		n = len(states) - 1
		if states[n] == 8 or states[n] == 9 or states[n] == 4:#no more space in front
			hlcontrol = 0.5*(base_speed - 0.2*base_speed) # turn around and backwards a bit to prevent stuck state
			hrcontrol = 0.5*(-base_speed - 0.2*base_speed)

		elif states[n] == 7:
			if front <= (PIPE_DIA/2 + 10):#100+20mm then turn
				turnstep = base_speed*0.33 #always turn right at T-junction
				hlcontrol = 0.25*(base_speed + turnstep)
				hrcontrol = 0.25*(base_speed - turnstep)
			elif front <= (PIPE_DIA/2 + 20) and front > (PIPE_DIA/2 + 10):#100+20mm then turn:
				turnstep = base_speed*0.1 #always turn right at T-junction
				hlcontrol = 0.6*(base_speed + turnstep)
				hrcontrol = 0.6*(base_speed - turnstep)
			else:
				turnstep = base_speed*0.1 #always turn right at T-junction
				hlcontrol = base_speed + turnstep
				hrcontrol = base_speed - turnstep

		elif states[n] == 6:#right corner/turn
			if front <= (PIPE_DIA/2 + 15):#115mm, then turn
				turnstep = base_speed*0.4 #always turn right
				hlcontrol = 0.25*(base_speed + turnstep)
				hrcontrol = 0.25*(base_speed - turnstep)
			elif front > (PIPE_DIA/2 + 10) and front <= PIPE_DIA*1.1:
				turnstep = base_speed*0.1 #always turn right
				hlcontrol = 0.6*(base_speed + turnstep)
				hrcontrol = 0.6*(base_speed - turnstep)
			else:#detect junctions at distance PIPE_DIA*1.2=240-->approach slowly
				hlcontrol = base_speed*0.6
				hrcontrol = base_speed*0.6

		elif states[n] == 5:#only way is to left turn-->must turn left
			if front <= (PIPE_DIA/2 + 15):
				turnstep = base_speed*0.4
				hlcontrol = 0.25*(base_speed - turnstep)
				hrcontrol = 0.25*(base_speed + turnstep)
			elif front > (PIPE_DIA/2 + 15) and front <= PIPE_DIA*1.1:
				turnstep = base_speed*0.15
				hlcontrol = 0.6*(base_speed - turnstep)
				hrcontrol = 0.6*(base_speed + turnstep)
			else:#detect junctions at distance PIPE_DIA*1.2=240-->approach slowly
				hlcontrol = base_speed*0.6
				hrcontrol = base_speed*0.6

		elif states[n] == 3:#right branch
			turnstep = base_speed*0.2 #always turn right
			hlcontrol = 0.66*(base_speed + turnstep)
			hrcontrol = 0.66*(base_speed - turnstep)


		elif states[n] == 10:#robot is stuck or need more infor to decide
			turnstep = base_speed*0.05#increase control force
			# sum_hlcontrol = sum_hlcontrol - turnstep
			# sum_hrcontrol = sum_hrcontrol - turnstep
			rotatestep = 0.5*base_speed
			hlcontrol = -0.1*base_speed + rotatestep
			hrcontrol = -0.1*base_speed - rotatestep

		else:#for cases<=2, controller will take values in lowcontrol function
			hlcontrol = 0
			hrcontrol = 0

	# print("hleft",hlcontrol)
	# print("hright",hrcontrol)
	# end of high level control algorithm
	lowcontrol(hlcontrol, hrcontrol, camcontrol, front, left, right, roll, pitch)

def lowcontrol(hlcontrol, hrcontrol, camcontrol, front, left, right, roll, pitch):
	global measureON
	if measureON == True:
		flcontrol = hlcontrol
		frcontrol = hlcontrol
	else:
		n = len(states) - 1
		diffdrive = (left - right)/SIDEDIFF_MAX #normalized by max difference
		# diffdrive>0 means robot is near right than left (right-sided)-->move right faster than
		# print("lidar:", diffdrive)	
		if diffdrive > 0.6*base_speed: #this means robot's rpy is wrong
			diffdrive = 0.6*base_speed
		elif diffdrive < -0.6*base_speed:
			diffdrive = -0.6*base_speed
		
		pchange = 2*pitch #when robot is left inclined its y axis--move right wheel faster
		if pchange > 0.8*base_speed: #this means robot's rpy is wrong
			pchange = 0.8*base_speed
		elif pchange < -0.8*base_speed:
			pchange = -0.8*base_speed
		if states[n] <= 2 or states[n] == 11:

			if right>STEERATCROSS: #suspect that there is a right branch as well
				turn2rightbranch = 0.33*base_speed
			else:
				turn2rightbranch = 0
			if states[n] == 2:#left branch, don't steer with lidar
				basecontrol = base_speed
				uneven_compensate = base_speed*0.05
				flcontrol = basecontrol  + pchange + uneven_compensate + turn2rightbranch
				frcontrol = basecontrol  - pchange - uneven_compensate - turn2rightbranch
			elif states[n] == 1:#sided to one side of pipe-->keep speed, steer back
				basecontrol = base_speed
				# flcontrol = basecontrol - 2*diffdrive + 0.75*pchange + turn2rightbranch
				# frcontrol = basecontrol + 2*diffdrive - 0.75*pchange - turn2rightbranch
				flcontrol = basecontrol - 2*diffdrive + 0.4*pchange + turn2rightbranch
				frcontrol = basecontrol + 2*diffdrive - 0.4*pchange - turn2rightbranch
			elif states[n] == 0:#at centerline -->accelerate
				basecontrol = base_speed*1.5
				flcontrol = basecontrol - diffdrive + pchange + turn2rightbranch
				frcontrol = basecontrol + diffdrive - pchange - turn2rightbranch

			elif states[n]==11:	#state==11 means robot is up/down hill or pointing to wall
				basecontrol = -base_speed	#backward a little
				flcontrol = basecontrol - diffdrive + pchange + base_speed
				frcontrol = basecontrol + diffdrive - pchange - base_speed
				
		else:#for all cases>2, take control values from highcontrol functions
			flcontrol = hlcontrol - 0.2*diffdrive + 1.2*pchange
			frcontrol = hrcontrol + 0.2*diffdrive - 1.2*pchange
	# print("ll",flcontrol)
	# print("lr",frcontrol)
	motor_commands(flcontrol, frcontrol, camcontrol)

def motor_commands(flcontrol, frcontrol,camcontrol):
	#always make two tail wheels depend on front wheels-coz of whegbot design
	# flcontrol = 2
	# frcontrol = 2
	if flcontrol > CONTROL_LIMIT:
		flcontrol = CONTROL_LIMIT
	elif flcontrol <- CONTROL_LIMIT:
		flcontrol = -CONTROL_LIMIT
	if frcontrol > CONTROL_LIMIT:
		frcontrol = CONTROL_LIMIT
	elif frcontrol <- CONTROL_LIMIT:
		frcontrol = -CONTROL_LIMIT
	blcontrol = flcontrol
	brcontrol = frcontrol
	#Re-initializing publishers from before since in a different function
	pub1 = rospy.Publisher('/mybot/joint1_velocity_controller/command', Float64, queue_size=100)
	pub2 = rospy.Publisher('/mybot/joint2_velocity_controller/command', Float64, queue_size=100)
	pub3 = rospy.Publisher('/mybot/joint3_velocity_controller/command', Float64, queue_size=100)
	pub4 = rospy.Publisher('/mybot/joint4_velocity_controller/command', Float64, queue_size=100)
	pub5 = rospy.Publisher('/mybot/joint5_velocity_controller/command', Float64, queue_size=100)
	pub6 = rospy.Publisher('/mybot/joint6_velocity_controller/command', Float64, queue_size=100)
	pub7 = rospy.Publisher('/mybot/joint7_position_controller/command', Float64, queue_size=100)
	#Publishing new torques to topics
	pub1.publish(flcontrol) 
	pub2.publish(frcontrol) 
	pub3.publish(blcontrol) 
	pub4.publish(brcontrol) 
	pub5.publish(blcontrol) 
	pub6.publish(brcontrol)
	pub7.publish(camcontrol)

#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
	try: mybot_joint_velocity_publisher()
	except rospy.ROSInterruptException: pass
