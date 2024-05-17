#!/usr/bin/env python
import rospy
import sys

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf_conversions import transformations

from quest2ros.msg import OVR2ROSInputs, OVR2ROSHapticFeedback
from geometry_msgs.msg import PoseStamped, Twist, geometry_msgs
import numpy as np 



class ros2quest:

  def __init__(self):

    # Subscribers
    self.ovr2ros_right_hand_pose_sub = rospy.Subscriber("/q2r_right_hand_pose", PoseStamped, self.ovr2ros_right_hand_pose_callback)
    self.ovr2ros_right_hand_twist_sub = rospy.Subscriber("/q2r_right_hand_twist", Twist, self.ovr2ros_right_hand_twist_callback)
    self.ovr2ros_right_hand_inputs_sub = rospy.Subscriber("/q2r_right_hand_inputs", OVR2ROSInputs, self.ovr2ros_right_hand_inputs_callback)
    self.ovr2ros_left_hand_pose_sub = rospy.Subscriber("/q2r_left_hand_pose", PoseStamped, self.ovr2ros_left_hand_pose_callback)
    self.ovr2ros_left_hand_twist_sub = rospy.Subscriber("/q2r_left_hand_twist", Twist, self.ovr2ros_left_hand_twist_callback)
    self.ovr2ros_left_hand_inputs_sub = rospy.Subscriber("/q2r_left_hand_inputs", OVR2ROSInputs, self.ovr2ros_left_hand_inputs_callback)
    
    # Publishers  
    self.ros2ovr_right_hand_haptic_feedback_pub = rospy.Publisher("/q2r_right_hand_haptic_feedback", OVR2ROSHapticFeedback, queue_size=1)
    self.ros2ovr_left_hand_haptic_feedback_pub = rospy.Publisher("/q2r_left_hand_haptic_feedback", OVR2ROSHapticFeedback, queue_size=1)

    self.right_hand = PoseStamped()
    self.right_hand_twist = Twist()
    self.right_hand_inputs = OVR2ROSInputs()

    self.left_hand_pose = PoseStamped()
    self.left_hand_twist = Twist()
    self.left_hand_inputs = OVR2ROSInputs()
    
    # TF broadcaster
    self.br = TransformBroadcaster()
    
    # Frame to reference joystick motion
    self.reference_frame = PoseStamped()
    
    # Simulating the robot EE with a static TF
    br_stat = StaticTransformBroadcaster()
    
    static_ee_transform = geometry_msgs.msg.TransformStamped()
    static_ee_transform.child_frame_id = "ee_robot"
    static_ee_transform.header.frame_id = "world"
    static_ee_transform.header.stamp = rospy.Time.now()
    static_ee_transform.transform.translation.x = 1.0
    static_ee_transform.transform.translation.y = 0.0
    static_ee_transform.transform.translation.z = 0.0
    quat = transformations.quaternion_from_euler(0, 0, 0)
    static_ee_transform.transform.rotation.x = quat[0]
    static_ee_transform.transform.rotation.y = quat[1]
    static_ee_transform.transform.rotation.z = quat[2]
    static_ee_transform.transform.rotation.w = quat[3]
    
    br_stat.sendTransform(static_ee_transform)
    # Simulating the robot EE with a static TF
    
    # Delta
    self.deltaTranslation = geometry_msgs.msg.Vector3()
    self.deltaRotation = geometry_msgs.msg.Quaternion()
    

  #####################
  ### Ros callbacks ###
  #####################
  # Right hand 
  def ovr2ros_right_hand_pose_callback(self, data):    
    self.right_hand = data
    
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "ee_robot"
    t.child_frame_id = "ee_robot_reference"
      
    if self.right_hand_inputs.button_lower == True:
      self.deltaTranslation.x = self.right_hand.pose.position.x - self.reference_frame.pose.position.x 
      self.deltaTranslation.y = self.right_hand.pose.position.y - self.reference_frame.pose.position.y 
      self.deltaTranslation.z = self.right_hand.pose.position.z - self.reference_frame.pose.position.z 
      
      q_ref = self.reference_frame.pose.orientation
      q_hand = self.right_hand.pose.orientation
      
      eul_ref = transformations.euler_from_quaternion([q_ref.x, q_ref.y, q_ref.z, q_ref.w])
      eul_hand = transformations.euler_from_quaternion([q_hand.x, q_hand.y, q_hand.z, q_hand.w])
      
      deltaRotationEul = tuple(i-j for i,j in zip(eul_hand, eul_ref))      
      deltaQuaternion = transformations.quaternion_from_euler(ai=deltaRotationEul[0], aj=deltaRotationEul[1], ak=deltaRotationEul[2])
      
      self.deltaRotation.x = deltaQuaternion[0]
      self.deltaRotation.y = deltaQuaternion[1]
      self.deltaRotation.z = deltaQuaternion[2]
      self.deltaRotation.w = deltaQuaternion[3]
      
    else:
      self.deltaTranslation.x = 0.0
      self.deltaTranslation.y = 0.0
      self.deltaTranslation.z = 0.0
      self.deltaRotation.x = 0.0
      self.deltaRotation.y = 0.0
      self.deltaRotation.z = 0.0
      self.deltaRotation.w = 1.0
    

    t.transform.translation = self.deltaTranslation
    t.transform.rotation = self.deltaRotation
    
    self.br.sendTransform(t)

  def ovr2ros_right_hand_twist_callback(self, data):    
    self.right_hand_twist = data

  def ovr2ros_right_hand_inputs_callback(self, data):
    
    # Upper front -> save relative pose
    if self.right_hand_inputs.button_lower == False and data.button_lower == True:
      self.reference_frame.pose.position = geometry_msgs.msg.Point(self.right_hand.pose.position.x, self.right_hand.pose.position.y, self.right_hand.pose.position.z)
      
      orientation = (self.right_hand.pose.orientation.x, self.right_hand.pose.orientation.y, self.right_hand.pose.orientation.z, self.right_hand.pose.orientation.w)
      # orientation_inv = transformations.quaternion_inverse(orientation)
      self.reference_frame.pose.orientation = geometry_msgs.msg.Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
      
    self.right_hand_inputs = data

  # Left hand 
  def ovr2ros_left_hand_pose_callback(self, data):    
    self.left_hand_pose = data

  def ovr2ros_left_hand_twist_callback(self, data):    
    self.left_hand_twist = data

  def ovr2ros_left_hand_inputs_callback(self, data):    
    self.left_hand_inputs = data

def main(args):
  rospy.init_node('quest2rosdemo', anonymous=True)

  r2q = ros2quest()
  rospy.sleep(1)

  r = rospy.Rate(300) 

  while not rospy.is_shutdown():
    r.sleep()


if __name__ == '__main__':
    main(sys.argv)