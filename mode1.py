#!/usr/bin/env python
import rospy
import time
import numpy as np
from math import degrees
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import sys
from std_msgs.msg import String
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from op3_walking_module_msgs.msg import WalkingParam
from robotis_controller_msgs.msg import StatusMsg
from sensor_msgs.msg import Imu

class Model:
	def __init__(self):
		self.a=0
		self.cntjln=0
			
		rospy.Subscriber("/robotis/open_cr/button", String, self.button_callback)   
		rospy.Subscriber("/robotis/status", StatusMsg, self.flag_callback)
		self.pubWalk=rospy.Publisher("/robotis/walking/set_params",WalkingParam,queue_size=5)
		self.pubWalks=rospy.Publisher("/robotis/walking/command",String,queue_size=5)
		self.pubEnableCtrl=rospy.Publisher("/robotis/enable_ctrl_module",String,queue_size=4)	
		
		#Set Param Walking
		self.walk = WalkingParam()
		self.walk.init_x_offset=(-0.00300000002608)
		self.walk.init_y_offset=(0.0299999993294)
		self.walk.init_z_offset=(0.019999999553)
		self.walk.init_roll_offset=(0.139626339078)
		self.walk.init_pitch_offset=(0.0349065847695)
		self.walk.init_yaw_offset=(0.0)
		self.walk.period_time=(2.0)
		self.walk.dsp_ratio=(0.40000000596)
		self.walk.step_fb_ratio=(0.40000000596)
		self.walk.x_move_amplitude=(0.019999999553)
		self.walk.y_move_amplitude=(0.0)
		self.walk.z_move_amplitude=(0.035000000149)
		self.walk.angle_move_amplitude=(-0.00)
		self.walk.move_aim_on=(True)
		self.walk.balance_enable=(True)
		self.walk.balance_hip_roll_gain=(0.34999999404)
		self.walk.balance_knee_gain=(0.10000000149)
		self.walk.balance_ankle_roll_gain=(0.10000000149)
		self.walk.balance_ankle_pitch_gain=(0.5)
		self.walk.y_swap_amplitude=(0.035000000149)
		self.walk.z_swap_amplitude=(0.0149999996647)
		self.walk.arm_swing_gain=(0.699999988079)
		self.walk.pelvis_offset=(0.11344639957)
		self.walk.hip_pitch_offset=(0.296705961227)
		self.walk.p_gain=(0)
		self.walk.i_gain=(2)
		self.walk.d_gain=(2)
		
		self.walk1 = WalkingParam()
		self.walk1.init_x_offset=(-0.00300000002608)
		self.walk1.init_y_offset=(0.0299999993294)
		self.walk1.init_z_offset=(0.019999999553)
		self.walk1.init_roll_offset=(0.139626339078)
		self.walk1.init_pitch_offset=(0.0349065847695)
		self.walk1.init_yaw_offset=(0.0)
		self.walk1.period_time=(2.0)
		self.walk1.dsp_ratio=(0.40000000596)
		self.walk1.step_fb_ratio=(0.40000000596)
		self.walk1.x_move_amplitude=(0.019999999553)
		self.walk1.y_move_amplitude=(0.0)
		self.walk1.z_move_amplitude=(0.035000000149)
		self.walk1.angle_move_amplitude=(0.02)
		self.walk1.move_aim_on=(True)
		self.walk1.balance_enable=(True)
		self.walk1.balance_hip_roll_gain=(0.34999999404)
		self.walk1.balance_knee_gain=(0.10000000149)
		self.walk1.balance_ankle_roll_gain=(0.10000000149)
		self.walk1.balance_ankle_pitch_gain=(0.5)
		self.walk1.y_swap_amplitude=(0.035000000149)
		self.walk1.z_swap_amplitude=(0.0149999996647)
		self.walk1.arm_swing_gain=(0.699999988079)
		self.walk1.pelvis_offset=(0.11344639957)
		self.walk1.hip_pitch_offset=(0.296705961227)
		self.walk1.p_gain=(0)
		self.walk1.i_gain=(2)
		self.walk1.d_gain=(2)	
		
		self.walk2 = WalkingParam()
		self.walk2.init_x_offset=(0.00300000002608)
		self.walk2.init_y_offset=(0.0149999996647)
		self.walk2.init_z_offset=(0.019999999553)
		self.walk2.init_roll_offset=(0.174532920122)
		self.walk2.init_pitch_offset=(0.0349065847695)
		self.walk2.init_yaw_offset=(0.0)
		self.walk2.period_time=(1.79999995232)
		self.walk2.dsp_ratio=(0.40000000596)
		self.walk2.step_fb_ratio=(0.40000000596)
		self.walk2.x_move_amplitude=(0.0)
		self.walk2.y_move_amplitude=(-0.0169999996647)
		self.walk2.z_move_amplitude=(0.0599999986589)
		self.walk2.angle_move_amplitude=(0)
		self.walk2.move_aim_on=(True)
		self.walk2.balance_enable=(True)
		self.walk2.balance_hip_roll_gain=(0.34999999404)
		self.walk2.balance_knee_gain=(0.10000000149)
		self.walk2.balance_ankle_roll_gain=(0.10000000149)
		self.walk2.balance_ankle_pitch_gain=(0.5)
		self.walk2.y_swap_amplitude=(0.035000000149)
		self.walk2.z_swap_amplitude=(0.00499999988824)
		self.walk2.arm_swing_gain=(0.699999988079)
		self.walk2.pelvis_offset=(0.0261799395084)
		self.walk2.hip_pitch_offset=(0.296705961227)
		self.walk2.p_gain=(0)
		self.walk2.i_gain=(2)
		self.walk2.d_gain=(2)
		
	#Subs Button	
	def button_callback(self,data):
		self.button_status = data.data
		if(self.button_status == "start"):
			self.btn1 = 1
		if(self.button_status == "mode"):
			self.btn1 = 2	
	#Subs Status	
	def flag_callback(self,data):
		self.flag_status = data.status_msg
		if(self.flag_status=="Stop walking"):
			self.cntjln += 1
			time.sleep(4.5)
			self.cmd_pub.publish(1)
	def jalan(self,param,t):
		time.sleep(1.5)
		self.pubWalk.publish(param)
		time.sleep(1)
		self.pubWalks.publish("start")
		time.sleep(t)
		self.pubWalks.publish("stop")		
	def run(self):
		if(self.btn1==1 and self.a==0):
			self.pubEnableCtrl.publish("walking_module")
			time.sleep(1)
			self.pubWalks.publish("stop")	
			self.a=1
		elif(self.a==1 and self.cntjln==1 and self.btn1==2):
			self.jalan(self.walk,4)
			time.sleep(4.9)
			self.a=2
		elif(self.a==2 and self.cntjln==2):
			time.sleep(5)
			self.jalan(self.walk,4)	
			self.a=3 
		elif(self.a==3 and self.cntjln==3):
			time.sleep(5)
			self.jalan(self.walk1,4)	
			self.a=4 
		elif(self.a==4 and self.cntjln==4):
			time.sleep(5)
			self.jalan(self.walk,4)	
			self.a=5 
		elif(self.a==5 and self.cntjln==5):
			time.sleep(5)
			self.jalan(self.walk1,4)	
			self.a=6 
		elif(self.a==6 and self.cntjln==6):
			time.sleep(5)
			self.jalan(self.walk,4)	
			self.a=7 
		elif(self.a==7 and self.cntjln==7):
			time.sleep(5)
			self.jalan(self.walk,4)	
			self.a=8 
		elif(self.a==8 and self.cntjln==8):
			time.sleep(5)
			self.jalan(self.walk,4)
			self.a=9
		elif(self.a==9 and self.cntjln==9):
			time.sleep(5)
			self.jalan(self.walk,4)	
			self.a=10 
		elif(self.a==10 and self.cntjln==10):
			time.sleep(5)
			self.jalan(self.walk1,4)	
			self.a=11 
		elif(self.a==11 and self.cntjln==11):
			time.sleep(5)
			self.jalan(self.walk,4)	
			self.a=12 
		elif(self.a==12 and self.cntjln==12):
			time.sleep(5)
			self.jalan(self.walk,3)	
			self.a=13 												
		print("X=",self.x,"Y=",self.y,"TH=",self.z)
		
if __name__ == '__main__':#untuk terminate
	rospy.init_node('robot_one')
	mod = Model()
	try:
		
		while not rospy.is_shutdown():
			mod.run()
	except rospy.ROSInterruptException():
		pass
