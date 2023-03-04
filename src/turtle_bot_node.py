#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import actionlib
import random
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from actionlib_msgs.msg import *

LINEAR_VEL = 0.22
STOP_DISTANCE = 2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
Obstacle_detected = False
MAX_LIDAR_DISTANCE = 3.5
class Obstacle():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")
        self.move_base.wait_for_server(rospy.Duration(5))
        self.explore()
        # self.obstacle()

    def explore(self):
        print("inside explore")

        goal_x = MAX_LIDAR_DISTANCE*random.random()
        goal_y = MAX_LIDAR_DISTANCE*random.random()
        quat_r1 = 0.0
        quat_r2 = 0.0
        quat_r3 = 0.0
        quat_r4 = 0.0
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(goal_x, goal_y, 0.000), Quaternion(quat_r1, quat_r2, quat_r3, quat_r4))
        self.move_base.send_goal(goal)
        print("goal given")
        success = self.move_base.wait_for_result(rospy.Duration(60)) 
        state = self.move_base.get_state()
        result = False
        if success and state == GoalStatus.SUCCEEDED:
            goal_x = goal_x+MAX_LIDAR_DISTANCE*random()
            goal_y = goal_y+MAX_LIDAR_DISTANCE*random()
        else:
            self.explore()

    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
       
        samples = len(scan.ranges)  # The number of samples is defined in 
                                    # turtlebot3_<model>.gazebo.xacro file,
                                    # the default is 360.
        samples_view = 1            # 1 <= samples_view <= samples
        
        if samples_view > samples:
            samples_view = samples

        if samples_view == 1:
            scan_filter.append(scan.ranges[0])

        else:
            left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view//2
            
            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = MAX_LIDAR_DISTANCE
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0
        
        return scan_filter

    def obstacle(self):
        twist = Twist()
        turtlebot_moving = True

        while not rospy.is_shutdown():
            lidar_distances = self.get_scan()
            min_distance = min(lidar_distances)

            if min_distance < SAFE_STOP_DISTANCE:
                if turtlebot_moving:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self._cmd_pub.publish(twist)
                    turtlebot_moving = False
                    rospy.loginfo('Stop!')
            else:
                twist.linear.x = LINEAR_VEL
                twist.angular.z = 0.0
                self._cmd_pub.publish(twist)
                turtlebot_moving = True
                rospy.loginfo('Distance of the obstacle : %f', min_distance)


def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()