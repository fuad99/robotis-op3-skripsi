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
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from op3_walking_module_msgs.msg import WalkingParam
from sensor_msgs.msg import JointState
from robotis_controller_msgs.msg import StatusMsg
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class Model:
	
	def __init__(self):
		
		self.a=0
		self.cntjln=0
		self.btn1=0
		self.btn2=0
		
		rospy.Subscriber("/robotis/open_cr/button", String, self.button_callback)   
		rospy.Subscriber("/robotis/status", StatusMsg, self.flag_callback)
		rospy.Subscriber('/imu/data', Imu, self.imu_callback)
		rospy.Subscriber('/odom_rf2o', Odometry, self.odom)

		self.pubWalk=rospy.Publisher("/robotis/walking/set_params",WalkingParam,queue_size=5)
		self.pubWalks=rospy.Publisher("/robotis/walking/command",String,queue_size=5)
		self.pubEnableCtrl=rospy.Publisher("/robotis/enable_ctrl_module",String,queue_size=4)
		self.cmd_pub=rospy.Publisher("/cmd",Int32,queue_size=5)
		self.odom=rospy.Publisher("/odom_pub",Odometry,queue_size=5)
		
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
		
		self.tr=0
		self.posex=0.0
		self.posey=0.0
		self.orientasi=Quaternion()
		self.twis=Twist()
		self.x=0
		self.y=0
		self.z=0
		self.go = Odometry()
		self.pub=Odometry()
		self.set=0
	def odom(self,data):
		
		self.go.pose.pose.position.x=data.pose.pose.position.x
		self.go.pose.pose.position.y=data.pose.pose.position.y
		self.orientasi=data.pose.pose.orientation
		self.z=data.pose.pose.orientation.z
		self.go.pose.covariance=data.pose.covariance
		self.twis=data.twist.twist
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
			time.sleep(2.5)
			self.tr=1
			
	def odom_pub(self,target,targety):
		
		
		self.y=targety
		
		#self.set=self.go.pose.pose.position.x-target
		#if(self.set>0):
		#	self.set=self.set
		#else:
			#self.set=abs(self.set)	
		self.x=target+self.go.pose.pose.position.x
		self.pub.header.stamp=rospy.get_rostime()
		self.pub.header.frame_id= "odom_laser"
		self.pub.child_frame_id= "base_link"
		self.pub.pose.pose.position=Point(self.x, self.y, 0)
		self.pub.pose.covariance=self.go.pose.covariance
		self.pub.pose.pose.orientation=self.orientasi
		self.pub.twist.twist=self.twis
		self.odom.publish(self.pub)
	def imu_callback(self, data):
		orientation_q = data.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)
		#print(self.yaw)
	def cek_hadap(self):
		if(self.yaw <=-0.1):
			self.walk.angle_move_amplitude=(-0.01)	
		else:
			self.walk.angle_move_amplitude=(0.005)
	def jalan(self,param,t):
		#self.cek_hadap()
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
			self.odom_pub(0,0)
			#print("update")
			time.sleep(1)
			self.tr=0
			self.a=1
		elif(self.a==1 and self.cntjln==1 and self.btn1==2):
			self.jalan(self.walk,4)
			print("jalan")
			if(self.tr==1):
				time.sleep(4.9)
				self.odom_pub(0.087,0)
				self.tr=0
			self.a=2
		elif(self.a==2 and self.cntjln==2):
			time.sleep(5)
			self.jalan(self.walk,4)	
			if(self.tr==1):
				time.sleep(4.9)
				self.odom_pub(0.095,0.0)
				self.tr=0
			self.a=3 
		elif(self.a==3 and self.cntjln==3):
			time.sleep(5)
			self.jalan(self.walk1,4)	
			if(self.tr==1):
				time.sleep(4.9)
				self.odom_pub(0.11,0.0)
				self.tr=0
			self.a=4 
		elif(self.a==4 and self.cntjln==4):
			time.sleep(5)
			
			self.jalan(self.walk1,4)	
			if(self.tr==1):
				time.sleep(4.9)
				self.odom_pub(0.07,-0.00)
				self.tr=0
			self.a=5 
		elif(self.a==5 and self.cntjln==5):
			time.sleep(5)
			if(self.x>=0.7):
				self.jalan(self.walk2,8)	
				if(self.tr==1):
					time.sleep(4.9)
					self.odom_pub(0.0,-0.15)
					self.tr=0
				self.a=6 
			
		elif(self.a==6 and self.cntjln==6):
			time.sleep(5)
			self.jalan(self.walk1,4)	
			if(self.tr==1):
				time.sleep(4.9)
				self.odom_pub(-0.05,-0.1)
				self.tr=0
			self.a=7 
		elif(self.a==7 and self.cntjln==7):
			time.sleep(5)
			self.jalan(self.walk1,4)	
			if(self.tr==1):
				time.sleep(4.9)
				self.odom_pub(-0.0825,-0.05)
				self.tr=0
			self.a=8 
		elif(self.a==8 and self.cntjln==8):
			time.sleep(5)
			self.jalan(self.walk,4)	
			if(self.tr==1):
				time.sleep(4.9)
				self.odom_pub(-0.167,-0.05)
				self.tr=0
			self.a=9
		elif(self.a==9 and self.cntjln==9):
			time.sleep(5)
			self.jalan(self.walk,4)	
			if(self.tr==1):
				time.sleep(4.9)
				self.odom_pub(-0.26,-0.05)
				self.tr=0
			self.a=10 
		elif(self.a==10 and self.cntjln==10):
			time.sleep(5)
			self.jalan(self.walk1,4)	
			if(self.tr==1):
				time.sleep(4.9)
				self.odom_pub(-0.315,-0.05)
				self.tr=0
			self.a=11 
		elif(self.a==11 and self.cntjln==11):
			time.sleep(5)
			self.jalan(self.walk,4)	
			if(self.tr==1):
				time.sleep(4.9)
				self.odom_pub(-0.43,-0.05)
				self.tr=0
			self.a=12 
		elif(self.a==12 and self.cntjln==12):
			time.sleep(5)
			self.jalan(self.walk,4)	
			if(self.tr==1):
				time.sleep(4.9)
				self.odom_pub(-0.53,-0.05)
				self.tr=0
			self.a=13 	
		elif(self.a==13 and self.cntjln==13):
			time.sleep(5)
			self.jalan(self.walk,3)	
			if(self.tr==1):
				time.sleep(4.9)
				self.odom_pub(-0.57,-0.05)
				self.tr=0
			self.a=14 						
							
							
		#self.x=self.go.pose.pose.position.x+0.05
		#self.y=self.go.pose.pose.position.y					
		print("X=",self.x,"Y=",self.y,"TH=",self.z)
if __name__ == '__main__':#untuk terminate
	rospy.init_node('robot_one')
	mod = Model()
	try:
		
		while not rospy.is_shutdown():
			mod.run()
	except rospy.ROSInterruptException():
		pass
