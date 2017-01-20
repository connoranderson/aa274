#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
import tf

class Controller:

    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

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

    def get_ctrl_output(self):
        # use self.x self.y and self.theta to compute the right control input here
        cmd_x_dot = 0.0 # forward velocity
        cmd_theta_dot = 0.0
        # end of what you need to modify
        cmd = Twist()
        cmd.linear.x = cmd_x_dot
        cmd.angular.z = cmd_theta_dot
        return cmd

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            ctrl_output = self.get_ctrl_output()
            self.pub.publish(ctrl_output)
            rate.sleep()

if __name__ == '__main__':
    ctrl = Controller()
    ctrl.run()
