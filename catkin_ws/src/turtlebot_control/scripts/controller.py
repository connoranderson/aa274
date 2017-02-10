#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import tf
import math 
import numpy as np

class Controller:


	def __init__(self):
		rospy.init_node('turtlebot_controller',anonymous=True)
		rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
		rospy.Subscriber('/turtlebot_control/position_goal', Float32MultiArray, self.callback_sp_pos)
		rospy.Subscriber('/turtlebot_control/velocity_goal', Float32MultiArray, self.callback_sp_vel)
		rospy.Subscriber('/turtlebot_control/control_mode', String, self.callback_sp_cmd)
		self.pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size = 10)
		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0
		# Goal Pose (Updated in callback_supervisor by supervisor node)
		self.goalPose = [0, 0, 0]
		self.goalVel = [0,0]
		self.cmdMode = 'vel-control'

	def callback(self, data):
		pose = data.pose[data.name.index("mobile_base")]
		twist = data.twist[data.name.index("mobile_base")]
		self.x = pose.position.x
		self.y = pose.position.y
		quaternion = (
			pose.orientation.x,
			pose.orientation.y,
			pose.orientation.z,
			pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.theta = euler[2]

	def callback_sp_pos(self, data):
		rospy.loginfo('received: %s' , data.data)
		self.goalPose = data.data

	def callback_sp_vel(self, data):
		rospy.loginfo('received: %s' , data.data)
		self.goalVel = data.data

	def callback_sp_cmd(self, data):
		rospy.loginfo('received: %s' , data.data)
		self.cmdMode = data.data

	def get_ctrl_output(self):
		# Use self.x self.y and self.theta to compute correct control input here
		#(x,y,th): current state
		#(x_g,y_g,th-g): desired final state

		# If supervisor is commanding velocity, just use goalVel directly.
		# Otherwise, let getPoseControl_output determine how to get to goal position.
		if self.cmdMode == 'pos-control':
			V,om = self.getPosControl_output()
		elif self.cmdMode == 'vel-control':
			V,om = self.goalVel
		else: # If we're not yet in a control mode, don't move
			V,om = self.goalVel

	    # Apply saturation limits
		V = np.sign(V)*min(0.5, np.abs(V))
		om = np.sign(om)*min(1, np.abs(om))

		cmd_x_dot = V # forward velocity
		cmd_theta_dot = om
		# end o f what you need to modify
		cmd = Twist ()
		cmd.linear.x = cmd_x_dot
		cmd.angular.z = cmd_theta_dot

		# # -------------- REMOVE THIS LINE -------------- #
		# print self.cmdMode


		return cmd

	def getPosControl_output(self):
		x = self.x
		y = self.y
		th = self.theta
		
		x_g = self.goalPose[0]
		y_g = self.goalPose[1]
		th_g = self.goalPose[2]


		# Define Controller Gains
		k1 = 0.8
		k2 = 0.5
		k3 = 0.5

		# Define relevant control parameters
		rho = math.sqrt((x-x_g)**2 + (y-y_g)**2)
		alpha = wrapToPi(math.atan2(y_g-y,x_g-x) - th)
		delta = wrapToPi(alpha + th - th_g)

		# Control Law
		V = k1*rho*math.cos(alpha)
		om = k2*alpha + k1*(np.sinc(2*alpha/np.pi))*(alpha + k3*delta)

		return V, om


	def run (self) :
		rate = rospy.Rate(10) # 10 Hz
		while not rospy.is_shutdown( ) :
			ctrl_output = self.get_ctrl_output()
			self.pub.publish(ctrl_output)
			rate.sleep()


def wrapToPi(a):
	if isinstance(a, list):    # backwards compatibility for lists (distinct from np.array)
		return [(x + np.pi) % (2*np.pi) - np.pi for x in a]
	return (a + np.pi) % (2*np.pi) - np.pi

if __name__ == '__main__':
		ctrl = Controller()
		ctrl.run()
