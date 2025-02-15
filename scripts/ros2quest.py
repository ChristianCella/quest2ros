#!/usr/bin/env python
import rospy
import sys

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener, LookupException, ConnectivityException
from tf_conversions import transformations
from quest2ros.msg import OVR2ROSInputs, OVR2ROSHapticFeedback
from geometry_msgs.msg import PoseStamped, Twist, geometry_msgs
import numpy as np

class ros2quest:

  '''Class constructor'''
  def __init__(self):

    # Time
    self.last_timestamp = rospy.Time.now()
    
    # TF broadcaster (in this code the 'desired_pose' frame is published)
    self.br = TransformBroadcaster()
    
    # TF listener  
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer)
    
    # Frame to reference joystick motion
    self.p_reference = np.zeros(3)
    self.q_reference = np.zeros(4)
    self.reference_name = "teleop_link"
    self.tcp_name = "dmp_link"
       
    # Try to get the robot end-effector pose, provided it is already available
    try:
      self.base2flange()
    except (LookupException, ConnectivityException):
      rospy.logerr("Exception catched during lookup transform... killing myself")
      rospy.signal_shutdown()
      
    ''' 
    Initialize subscribers
    Subscribe to:
    - /q2r_right_hand_pose: get the right hand pose
    - /q2r_right_hand_inputs: get the right hand inputs (buttons, triggers, etc.)
    '''
    self.ovr2ros_right_hand_pose_sub = rospy.Subscriber("/q2r_right_hand_pose", PoseStamped, self.ovr2ros_right_hand_pose_callback, queue_size=1)
    self.ovr2ros_right_hand_inputs_sub = rospy.Subscriber("/q2r_right_hand_inputs", OVR2ROSInputs, self.ovr2ros_right_hand_inputs_callback, queue_size=1)
    
    ''' 
    Initialize publishers
    Publish to:
    - /q2r_right_hand_haptic_feedback: send haptic feedback to the right hand
    - /q2r_left_hand_haptic_feedback: send haptic feedback to the left hand
    - /desired_pose: send the desired pose to the robot
    '''
    self.ros2ovr_right_hand_haptic_feedback_pub = rospy.Publisher("/q2r_right_hand_haptic_feedback", OVR2ROSHapticFeedback, queue_size=1)
    self.ros2ovr_left_hand_haptic_feedback_pub = rospy.Publisher("/q2r_left_hand_haptic_feedback", OVR2ROSHapticFeedback, queue_size=1)
    self.desired_pose_pub = rospy.Publisher("/desired_pose", PoseStamped, queue_size=1)

    # Store poses and inputs
    self.right_hand_position = np.zeros(3)
    self.right_hand_orientation = np.zeros(4)
    self.right_hand_inputs = OVR2ROSInputs()

  '''
  Class-specific methods
  '''
  
  def ovr2ros_right_hand_pose_callback(self, data): 
    ''' 
    Callback function for the right hand pose.
    Triggered whenever a new PoseStamped message arrives on the /q2r_right_hand_pose topic.
    '''   
    # self.right_hand_position = np.array([data.pose.position.y, -data.pose.position.x, data.pose.position.z])
    self.right_hand_position = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    self.right_hand_orientation = np.array([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
    now = rospy.Time.now()
    
    # Ensure increasing timestamps
    if now <= self.last_timestamp:
      now = self.last_timestamp + rospy.Duration(1e-9)  # Increment by a nanosecond
    self.last_timestamp = now
    
    # This desired pose will be broadcast into TF as 'desired_pose' relative to the reference frame
    desiredPose = geometry_msgs.msg.TransformStamped()
    desiredPose.header.stamp = now
    desiredPose.header.frame_id = self.reference_name
    desiredPose.child_frame_id = "desired_pose"
    
    # If button 'A' is pressed:
    if self.right_hand_inputs.button_lower == True:

      # 1) Compute the delta in terms of poses
      deltaPosition = self.right_hand_position - self.p_reference
      
      # 2) Compute the delta in terms of orientations
      q_ref_inv = transformations.quaternion_inverse(self.q_reference)
      deltaOrientation = transformations.quaternion_multiply(q_ref_inv, self.right_hand_orientation)

      # 3) Add the deltas to the robot's current base->flange pose  
      desiredPosition = self.base2flangePosition + deltaPosition
      desiredPoseOrientation = transformations.quaternion_multiply(deltaOrientation, self.base2flangeOrientation)
      
      # 4) Fill in the desired pose
      desiredPose.transform.translation.x = desiredPosition[0]
      desiredPose.transform.translation.y = desiredPosition[1]
      desiredPose.transform.translation.z = desiredPosition[2]  
      desiredPose.transform.rotation.x = desiredPoseOrientation[0]
      desiredPose.transform.rotation.y = desiredPoseOrientation[1]
      desiredPose.transform.rotation.z = desiredPoseOrientation[2]
      desiredPose.transform.rotation.w = desiredPoseOrientation[3]

    # Button 'A' is not pressed
    else:

      # Just set the transform to be equal to the robot's current base->flange pose
      try:
        self.base2flange()
        
      except (LookupException, ConnectivityException):
        rospy.logerr("Exception catched during lookup transform")
        return
      
      desiredPose.transform.translation.x = self.base2flangePosition[0]
      desiredPose.transform.translation.y = self.base2flangePosition[1]
      desiredPose.transform.translation.z = self.base2flangePosition[2]
      desiredPose.transform.rotation.x = self.base2flangeOrientation[0]
      desiredPose.transform.rotation.y = self.base2flangeOrientation[1]
      desiredPose.transform.rotation.z = self.base2flangeOrientation[2]
      desiredPose.transform.rotation.w = self.base2flangeOrientation[3]
    
    # Regardless of the button state, broadcast the desired pose
    self.br.sendTransform(desiredPose)
    
    # Publishing hand pose to topic (/desired_pose)
    # This is the same transform data as 'desired_pose', just in PoseStamped form
    msg = geometry_msgs.msg.PoseStamped()
    msg.header.stamp = now
    msg.header.frame_id = self.reference_name
    msg.pose.position.x = desiredPose.transform.translation.x
    msg.pose.position.y = desiredPose.transform.translation.y
    msg.pose.position.z = desiredPose.transform.translation.z
    msg.pose.orientation.x = desiredPose.transform.rotation.x
    msg.pose.orientation.y = desiredPose.transform.rotation.y
    msg.pose.orientation.z = desiredPose.transform.rotation.z
    msg.pose.orientation.w = desiredPose.transform.rotation.w
    self.desired_pose_pub.publish(msg)
    
  def ovr2ros_right_hand_inputs_callback(self, data):
    ''' 
    Callback function for the right hand inputs.
    Triggered whenever a new OVR2ROSInputs message arrives on the /q2r_right_hand_inputs topic.
    '''
    # When the button 'A' transitions from not pressed to pressed, store the current right hand pose as the reference
    if self.right_hand_inputs.button_lower == False and data.button_lower == True:
      self.p_reference = self.right_hand_position
      self.q_reference = self.right_hand_orientation
    
      # Getting robot's ee for create the new desired_pose reference frame
      try:
        self.base2flange()
        
      except (LookupException, ConnectivityException):
        rospy.logerr("Exception catched during lookup transform")
        return
    
    # Update the stored right hand inputs
    self.right_hand_inputs = data
    
          
  def base2flange(self):
    ''' 
    Get the transformation from the base to the flange.
    The lookup waits for 10 seconds before throwing an exception.
    '''
    base2flange = geometry_msgs.msg.TransformStamped()
    base2flange = self.tf_buffer.lookup_transform(self.reference_name, self.tcp_name, rospy.Time(), rospy.Duration(1.0))
    self.base2flangePosition = np.array([base2flange.transform.translation.x, base2flange.transform.translation.y, base2flange.transform.translation.z])
    self.base2flangeOrientation = np.array([base2flange.transform.rotation.x, base2flange.transform.rotation.y, base2flange.transform.rotation.z, base2flange.transform.rotation.w])

def main(args):
  rospy.init_node('quest2rosdemo', anonymous=True)

  r2q = ros2quest()

  r = rospy.Rate(100) 

  while not rospy.is_shutdown():
    r.sleep()


if __name__ == '__main__':
    main(sys.argv)