#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import tf
from tf.transformations import quaternion_from_euler

class MyTransformNode(object):
    def __init__(self):
        rospy.init_node('my_transform_node')

        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.laser_frame = rospy.get_param('~laser_frame', 'laser')
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_link')
        self.global_frame = rospy.get_param('~global_frame', 'map')
        self.rate = rospy.get_param('~rate', 10.0)

        self.tf_listener = tf.TransformListener()

        self.pose_publisher_base = rospy.Publisher('/tf_'+self.base_frame, PoseStamped, queue_size=10)
        self.pose_publisher_laser = rospy.Publisher('/tf_'+self.laser_frame, PoseStamped, queue_size=10)
        self.pose_publisher_camera = rospy.Publisher('/tf_'+self.camera_frame, PoseStamped, queue_size=10)

        self.map_to_base_rate = rospy.get_param('~map_to_base_rate', self.rate)
        self.map_to_laser_rate = rospy.get_param('~map_to_laser_rate', self.rate)
        self.map_to_camera_rate = rospy.get_param('~map_to_camera_rate', self.rate)

        rospy.Timer(rospy.Duration(1.0 / self.map_to_camera_rate), self.tf_to_poses_callback)

    def tf_to_poses_callback(self, event):
        try:
            robot_transform = self.tf_listener.lookupTransform(self.global_frame, self.base_frame, rospy.Time(0))
            robot_pose_msg = self.create_pose_msg(robot_transform)
            self.pose_publisher_base.publish(robot_pose_msg)

            camera_transform = self.tf_listener.lookupTransform(self.global_frame, self.camera_frame, rospy.Time(0))
            camera_pose_msg = self.create_pose_msg(camera_transform)
            self.pose_publisher_camera.publish(camera_pose_msg)

            laser_transform = self.tf_listener.lookupTransform(self.global_frame, self.laser_frame, rospy.Time(0))
            laser_pose_msg = self.create_pose_msg(laser_transform)
            self.pose_publisher_laser.publish(laser_pose_msg)

        except Exception as e:
            rospy.logerr(str(e))

    def create_pose_msg(self, transform):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.global_frame
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = transform[0][0]
        pose_msg.pose.position.y = transform[0][1]
        pose_msg.pose.position.z = transform[0][2]
        pose_msg.pose.orientation.x = transform[1][0]
        pose_msg.pose.orientation.y = transform[1][1]
        pose_msg.pose.orientation.z = transform[1][2]
        pose_msg.pose.orientation.w = transform[1][3]
        return pose_msg

def main():
    node = MyTransformNode()
    rospy.spin()

if __name__ == '__main__':
    main()