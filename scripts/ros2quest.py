#!/usr/bin/env python
import rospy
import sys

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener, LookupException, ConnectivityException
from tf_conversions import transformations
from quest2ros.msg import OVR2ROSInputs, OVR2ROSHapticFeedback
from geometry_msgs.msg import PoseStamped, Twist, geometry_msgs
import numpy as np 



class ros2quest:

  def __init__(self):

    # TF broadcaster
    self.br = TransformBroadcaster()
    
    # TF listener  
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer)
    
    # Frame to reference joystick motion
    self.reference_frame = PoseStamped()
    
    # ------------------------------------------------------ #
    # ------------------------ Debug ----------------------- #
    # ------------------------------------------------------ #
    # Simulating the robot EE with a static TF
    br_stat = StaticTransformBroadcaster()
    
    static_ee_transform = geometry_msgs.msg.TransformStamped()
    static_ee_transform.child_frame_id = "base"
    static_ee_transform.header.frame_id = "flange"
    static_ee_transform.header.stamp = rospy.Time.now()
    static_ee_transform.transform.translation.x = 0.0
    static_ee_transform.transform.translation.y = 0.0
    static_ee_transform.transform.translation.z = 0.0
    quat = transformations.quaternion_from_euler(0, 0, 0)
    static_ee_transform.transform.rotation.x = quat[0]
    static_ee_transform.transform.rotation.y = quat[1]
    static_ee_transform.transform.rotation.z = quat[2]
    static_ee_transform.transform.rotation.w = quat[3]
    
    br_stat.sendTransform(static_ee_transform)
    # ------------------------------------------------------ #
    # ------------------------ Debug ----------------------- #
    # ------------------------------------------------------ #
    
    # Getting robot's ee for create the new desired_pose reference frame
    self.base2flange = geometry_msgs.msg.TransformStamped()
    try:
      self.base2flange = self.tf_buffer.lookup_transform("base", "flange", rospy.Time(), rospy.Duration(1))
      
    except (LookupException, ConnectivityException):
      rospy.ERROR("Exception catched during lookup transform... killing myself")
      rospy.signal_shutdown()
      
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
    
    

  #####################
  ### Ros callbacks ###
  #####################
  # Right hand 
  def ovr2ros_right_hand_pose_callback(self, data):    
    self.right_hand = data
    
    desiredPose = geometry_msgs.msg.TransformStamped()
    desiredPose.header.stamp = rospy.Time.now()
    desiredPose.header.frame_id = "base"
    desiredPose.child_frame_id = "desired_pose"
    
    if self.right_hand_inputs.button_lower == True:
      delta = geometry_msgs.msg.Transform()
      delta.translation.x = self.right_hand.pose.position.x - self.reference_frame.pose.position.x 
      delta.translation.y = self.right_hand.pose.position.y - self.reference_frame.pose.position.y 
      delta.translation.z = self.right_hand.pose.position.z - self.reference_frame.pose.position.z 
      
      q_ref = self.reference_frame.pose.orientation
      q_hand = self.right_hand.pose.orientation
      
      eul_ref = transformations.euler_from_quaternion([q_ref.x, q_ref.y, q_ref.z, q_ref.w])
      eul_hand = transformations.euler_from_quaternion([q_hand.x, q_hand.y, q_hand.z, q_hand.w])
      
      deltaRotationEul = tuple(i-j for i,j in zip(eul_hand, eul_ref))      
      deltaQuaternion = transformations.quaternion_from_euler(ai=deltaRotationEul[0], aj=deltaRotationEul[1], ak=deltaRotationEul[2])
      
      delta.rotation.x = deltaQuaternion[0]
      delta.rotation.y = deltaQuaternion[1]
      delta.rotation.z = deltaQuaternion[2]
      delta.rotation.w = deltaQuaternion[3]
      
      # rospy.loginfo("Delta translation: [%s, %s, %s]", delta.translation.x, delta.translation.y, delta.translation.z)
      # rospy.loginfo("Delta rotation: [%s, %s, %s, %s]", delta.rotation.x, delta.rotation.y, delta.rotation.z, delta.rotation.w)
      
      desiredPose.transform.translation.x = self.base2flange.transform.translation.x + delta.translation.x
      desiredPose.transform.translation.y = self.base2flange.transform.translation.y + delta.translation.y
      desiredPose.transform.translation.z = self.base2flange.transform.translation.z + delta.translation.z
      
      desiredPose.transform.rotation.w = self.base2flange.transform.rotation.w * delta.rotation.w - self.base2flange.transform.rotation.x * delta.rotation.x - self.base2flange.transform.rotation.y * delta.rotation.y - self.base2flange.transform.rotation.z * delta.rotation.z
      desiredPose.transform.rotation.x = self.base2flange.transform.rotation.w * delta.rotation.x + self.base2flange.transform.rotation.x * delta.rotation.w + self.base2flange.transform.rotation.y * delta.rotation.z - self.base2flange.transform.rotation.z * delta.rotation.y
      desiredPose.transform.rotation.y = self.base2flange.transform.rotation.w * delta.rotation.y - self.base2flange.transform.rotation.x * delta.rotation.z + self.base2flange.transform.rotation.y * delta.rotation.w + self.base2flange.transform.rotation.z * delta.rotation.x
      desiredPose.transform.rotation.z = self.base2flange.transform.rotation.w * delta.rotation.z + self.base2flange.transform.rotation.x * delta.rotation.y - self.base2flange.transform.rotation.y * delta.rotation.x + self.base2flange.transform.rotation.z * delta.rotation.w

    else:
      try:
        self.base2flange = self.tf_buffer.lookup_transform("base", "flange", rospy.Time())
        
      except (LookupException, ConnectivityException):
        rospy.ERROR("Exception catched during lookup transform")
        return
      
      desiredPose.transform = self.base2flange.transform
    
    
    self.br.sendTransform(desiredPose)

  def ovr2ros_right_hand_twist_callback(self, data):    
    self.right_hand_twist = data

  def ovr2ros_right_hand_inputs_callback(self, data):
    
    # Upper front -> save relative pose
    if self.right_hand_inputs.button_lower == False and data.button_lower == True:
      self.reference_frame.pose.position = geometry_msgs.msg.Point(self.right_hand.pose.position.x, 
                                                                   self.right_hand.pose.position.y, 
                                                                   self.right_hand.pose.position.z)
      self.reference_frame.pose.orientation = geometry_msgs.msg.Quaternion(self.right_hand.pose.orientation.x, 
                                                                           self.right_hand.pose.orientation.y, 
                                                                           self.right_hand.pose.orientation.z, 
                                                                           self.right_hand.pose.orientation.w)
    
      # Getting robot's ee for create the new desired_pose reference frame
      try:
        self.base2flange = self.tf_buffer.lookup_transform("base", "flange", rospy.Time())
        
      except (LookupException, ConnectivityException):
        rospy.ERROR("Exception catched during lookup transform")
        return
          
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

  r = rospy.Rate(300) 

  while not rospy.is_shutdown():
    r.sleep()


if __name__ == '__main__':
    main(sys.argv)