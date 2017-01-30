#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray
import tf

class Supervisor:

  def __init__(self):
    rospy.init_node('turtlebot_supervisor', anonymous=True)
    rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
	  self.pos_sp_pub = rospy.Publisher('/turtlebot_controller/position_goal', Float32MultiArray, queue_size=10)
    self.trans_listener = tf.TransformListener()
    self.trans_broad = tf.TransformBroadcaster()
	  self.has_tag_location = False

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

  def run(self):
    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
      self.loop()
      rate.sleep()

if __name__ == '__main__':
  sup = Supervisor()
  sup.run()
