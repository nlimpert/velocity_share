#!/usr/bin/env python

import rospy
import copy
from velocity_share_msgs.msg import RobotVelInfo
from geometry_msgs.msg import PoseStamped

def vel_info_publisher():
    pub = rospy.Publisher('robot_velocities', RobotVelInfo, queue_size=10)
    rospy.init_node('robot_velocities_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        robot_velocities = RobotVelInfo()
        robot_velocities.robot_name = "Dummy"
        robot_velocities.high_prio = True
        robot_velocities.path.header.frame_id = "map"
        robot_velocities.path.header.stamp = rospy.get_rostime()
        cur_pose = PoseStamped()
        cur_pose.header.frame_id = "map"

        cur_pose.pose.orientation.x = 0.0
        cur_pose.pose.orientation.y = 0.0
        cur_pose.pose.orientation.z = 0.0
        cur_pose.pose.orientation.w = 1.0

        cur_pose.pose.position.x = 3.0
        cur_pose.pose.position.y = 4.0

        robot_velocities.path.poses.append(cur_pose)
        cur_pose = copy.deepcopy(cur_pose)
        cur_pose.pose.position.x = -2.0
        cur_pose.pose.position.y = 5.0

        robot_velocities.path.poses.append(cur_pose)
        cur_pose = copy.deepcopy(cur_pose)
        cur_pose.pose.position.x = -3.0
        cur_pose.pose.position.y = 6.0
        robot_velocities.path.poses.append(cur_pose)

        print robot_velocities

        pub.publish(robot_velocities)
        rate.sleep()

if __name__ == '__main__':
    try:
        vel_info_publisher()
    except rospy.ROSInterruptException:
        pass
