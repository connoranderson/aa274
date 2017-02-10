#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import tf
import numpy as np


class Supervisor:

  class RobotStates:
      INIT, FINDING_FLAG, TURNING_TO_FLAG, MOVING_TO_FLAG, STOPPED_AT_FLAG = range(5)

  def __init__(self):
    rospy.init_node('turtlebot_supervisor', anonymous=True)
    self.pos_sp_pub = rospy.Publisher('/turtlebot_control/position_goal', Float32MultiArray, queue_size=10)
    self.vel_sp_pub = rospy.Publisher('/turtlebot_control/velocity_goal', Float32MultiArray, queue_size=10)
    self.cmd_sp_pub = rospy.Publisher('/turtlebot_control/control_mode', String, queue_size=10)
    self.trans_listener = tf.TransformListener()
    self.trans_broad = tf.TransformBroadcaster()
    self.has_tag_location = False
    self.current_state = self.RobotStates.INIT
    rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)

  def gazebo_callback(self, data):
    pose = data.pose[data.name.index("mobile_base")]
    quaternion = (pose.orientation.x,
                  pose.orientation.y,
                  pose.orientation.z,
                  pose.orientation.w)
    self.trans_broad.sendTransform((pose.position.x,pose.position.y,0),
                    quaternion,
                    rospy.Time.now(),
                    "mobile_base",
                    "world")
    self.x = pose.position.x
    self.y = pose.position.y
    euler = tf.transformations.euler_from_quaternion(quaternion)
    self.theta = euler[2]

  def loop(self):
    try:
      # tf knows where the tag is
      (translation,rotation) = self.trans_listener.lookupTransform("/world", "/tag_0", rospy.Time(0))
      self.has_tag_location = True

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      # tf doesn't know where the tag is
      translation = (0,0,0)
      rotation = (0,0,0,1)
      self.has_tag_location = False

    ######################################
    # Your state machine logic goes here #
    ######################################

    # translation = (-1.659247268975817, 1.66515226012091, 0.26567725287112204)
    # rotation = (0.5712068984596882, -0.4191825198564738, 0.5684109029168972, 0.41823168177941494)


    if self.current_state == self.RobotStates.INIT:
      # Change control mode to velocity control
      self.switchControlMode('vel-control')
      # Spin in a circle
      self.publishTurn_vel(0.5)
      # Transition to FINDING_FLAG state
      self.current_state = self.RobotStates.FINDING_FLAG
    
    if self.current_state == self.RobotStates.FINDING_FLAG:
      # If flag has just been found, move towards it
      if self.has_tag_location == True:
        # print translation
        # print rotation

        # Switch to moving to tag
        self.current_state = self.RobotStates.MOVING_TO_FLAG
        
        # Publish desired goal location
        x_g = translation[0]
        y_g = translation[1]
        th_g = self.theta
        goalState = [x_g,y_g,th_g]
        self.publishGoal_pos(goalState)

        # Change control mode to position control
        self.switchControlMode('pos-control')

    # If we were in the middle of moving to the flag, change states to finding_flag again
    if self.current_state == self.RobotStates.MOVING_TO_FLAG:
      if self.has_tag_location == False:
        self.current_state = self.RobotStates.FINDING_FLAG
        # Change control mode to velocity control
        self.switchControlMode('vel-control')
        # Spin in a circle
        self.publishTurn_vel(0.5)
      distanceToFiducial = np.sqrt((translation[0]-self.x)**2 + (translation[1]-self.y)**2)
      # print distanceToFiducial

      if distanceToFiducial < 0.35:
        self.current_state = self.RobotStates.STOPPED_AT_FLAG
        self.switchControlMode('vel-control')
        self.publishTurn_vel(0.0)




  def switchControlMode(self,mode):
    if mode == 'vel-control':
      # Change control mode to position control
      msg = 'vel-control'   
      self.cmd_sp_pub.publish(msg)
    elif mode == 'pos-control':
      # Change control mode to position control
      msg = 'pos-control'   
      self.cmd_sp_pub.publish(msg)

  def publishTurn_vel(self,turnRate):
    # Spin in a circle
    goalState = [0,turnRate]
    msg = Float32MultiArray()
    msg.data = goalState
    self.vel_sp_pub.publish(msg)

  def publishGoal_pos(self, goalState):
    # # Publish desired goal location
    msg = Float32MultiArray()
    msg.data = goalState
    self.pos_sp_pub.publish(msg)


  def run(self):
    rate = rospy.Rate(10) # 10 Hz
    rospy.sleep(1) # Give publishers & subscribers time to connect
    while not rospy.is_shutdown():
      self.loop()
      rate.sleep()

if __name__ == '__main__':
  sup = Supervisor()
  sup.run()
