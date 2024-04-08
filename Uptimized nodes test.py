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
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

LINEAR_VEL = 0.05
ANGULAR_VEL = 0.05
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

class Obstacle():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.start_time = rospy.Time.now()
        self.obstacle()

    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []

        samples = len(scan.ranges)  # The number of samples is defined in
                                    # turtlebot3_<model>.gazebo.xacro file,
                                    # the default is 360.
        samples_view = 1            # 1 <= samples_view <= samples

        if samples_view > samples:
            samples_view = samples

        if samples_view is 1:
            scan_filter.append(scan.ranges[0])

        else:
            left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view//2

            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0

        return scan_filter
    

    def obstacle(self):
        twist = Twist()
        turtlebot_moving = True

        while not rospy.is_shutdown():
            lidar_distances = self.get_scan()
            min_distance = min(lidar_distances)

            rospy.loginfo('Minimum distance to obstacle: %f', min_distance)
            if min_distance < SAFE_STOP_DISTANCE:
                if turtlebot_moving:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self._cmd_pub.publish(twist)
                    turtlebot_moving = False
                    rospy.loginfo('Stop!')
                    time.sleep(1)

                    #back up
                    twist.linear.x = -LINEAR_VEL
                    twist.angular.z = 0.0
                    self._cmd_pub.publish(twist)
                    turtlebot_moving = True
                    time.sleep(1)
                    turtlebot_moving = False
                    lidar_distances = self.get_scan()
                    startdistance = min(lidar_distances)

                    # Determine direction to turn based on lidar data
                    # Turn right
                    twist.linear.x = 0.0
                    twist.angular.z = -0.1
                    rospy.loginfo('looking right')
                    self._cmd_pub.publish(twist)
                    time.sleep(1)
                    lidar_distances = self.get_scan()
                    rightdistance = min(lidar_distances)

                    if rightdistance < startdistance:
                     # Determine direction to turn
                        # Turn left
                        twist.linear.x = 0.0
                        twist.angular.z = 0.75
                        rospy.loginfo('Turning left')
                        rospy.loginfo('length %d', len(lidar_distances))
                    else:
                        # Turn right
                        twist.linear.x = 0.0
                        twist.angular.z = -0.75
                        rospy.loginfo('Turning right')
                        rospy.loginfo('length %d', len(lidar_distances))

                    # Publish twist message to make the turn
                    self._cmd_pub.publish(twist)
                    time.sleep(1)
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self._cmd_pub.publish(twist)

                    turtlebot_moving = True
                    time.sleep(1)

                    #turn
                    #twist.linear.x = 0.0
                    #twist.angular.z = 0.5
                    #self._cmd_pub.publish(twist)
                    #turtlebot_moving = True
                    #rospy.loginfo('Turning clockwise')
                    #lidar_distances = self.get_scan()
                    #min_distance = min(lidar_distances)
                    #time.sleep(1)
                    #twist.linear.x = 0.0
                    #twist.angular.z = 0.0
                    #turtlebot_moving = False
                    #self._cmd_pub.publish(twist)
                    #if min_distance < SAFE_STOP_DISTANCE:
                        #for i in range(3):
                    #    twist.linear.x = 0.0
                     #   twist.angular.z = -1.0
                      #  self._cmd_pub.publish(twist)
                       # turtlebot_moving = True
                        #rospy.loginfo('Turning counter clockwise')
                        #time.sleep(1)
            else:
                twist.linear.x = LINEAR_VEL
                twist.angular.z = 0.0
                self._cmd_pub.publish(twist)
                turtlebot_moving = True
                rospy.loginfo('Distance of the obstacle : %f', min_distance)

            # Check if two minutes have elapsed
            elapsed_time = rospy.Time.now() - self.start_time
            if elapsed_time.to_sec() >= 60:
                rospy.loginfo("One minutes have passed. Stopping the robot.")
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self._cmd_pub.publish(twist)
                break

def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
