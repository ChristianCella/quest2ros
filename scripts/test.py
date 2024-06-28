#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseStamped
import math
import time

class DesiredPosePublisher:
    def __init__(self):
        rospy.init_node('desired_pose_publisher', anonymous=True)
        self.pub = rospy.Publisher('/desired_pose', PoseStamped, queue_size=10)
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.rate = rospy.Rate(70)  # 70 Hz
        self.start_time = time.time()

    def get_end_effector_pose(self):
        try:
            (trans, rot) = self.listener.lookupTransform('tabletop', 'flange', rospy.Time(0))
            return trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF lookup failed")
            return None, None

    def publish_desired_pose(self):
        
        trans, rot = self.get_end_effector_pose()
        while trans is None or rot is None:
            trans, rot = self.get_end_effector_pose()
        
        while not rospy.is_shutdown():
            current_time = time.time()
            elapsed_time = current_time - self.start_time
            sine_wave = 0.2 * math.sin(2 * math.pi * 0.1 * elapsed_time)  # Sinusoidal signal with 1 Hz frequency and 0.1 amplitude

            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "tabletop"

            pose.pose.position.x = trans[0] 
            pose.pose.position.y = trans[1] + sine_wave
            pose.pose.position.z = trans[2]

            pose.pose.orientation.x = rot[0]
            pose.pose.orientation.y = rot[1]
            pose.pose.orientation.z = rot[2]
            pose.pose.orientation.w = rot[3]

            self.pub.publish(pose)   
            self.broadcaster.sendTransform((pose.pose.position.x, pose.pose.position.y, pose.pose.position.z), (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z ,pose.pose.orientation.w), rospy.Time.now(), "SWAG", "tabletop")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = DesiredPosePublisher()
        node.publish_desired_pose()
    except rospy.ROSInterruptException:
        pass
