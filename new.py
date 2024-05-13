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

import sys
sys.path.append('/home/ubuntu/catkin_ws2/src/turtlebot3/turtlebot3_example/src/turtlebot3_example/')


import rospy
import math
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from readFromLightSensor import *
import RPi.GPIO as GPIO


GPIO.setmode(GPIO.BCM)

LED = 25
LINEAR_VEL = 0.15
ANGULAR_VEL = 1
STOP_DISTANCE = 0.25
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
EMERGENCY_STOP_DISTANCE = 0.15 + LIDAR_ERROR

GPIO.setup(LED, GPIO.OUT)

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
        samples_view = 90            # 1 <= samples_view <= samples

        if samples_view > samples:
            samples_view = samples

        if samples_view is 1:
            scan_filter.append(scan.ranges[0])

        else:
            left_lidar_samples_ranges = -(samples_view//2) # removed + samples_view % 2
            right_lidar_samples_ranges = samples_view//2

            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 10
            elif scan_filter[i] == 0:
                scan_filter[i] = 10

        return scan_filter

    def obstacle(self):
        turtlebot_moving = True
        speed_updates = 0
        speed_accumulation = 0
        average_linear_speed = 0
        collision_counter = 0

        def updateVelocity(linear, angular, speed_updates, speed_accumulation):
            twist = Twist()
            twist.linear.x = LINEAR_VEL*linear
            twist.angular.z = ANGULAR_VEL*angular
            speed_updates+=1
            speed_accumulation=speed_accumulation + twist.linear.x
            self._cmd_pub.publish(twist)
            return speed_updates, speed_accumulation

        def direction():
            lidar_distances = self.get_scan()
            right = [x for x in lidar_distances[:36] if x != 10]
            left = [x for x in lidar_distances[36:] if x != 10]

            if (len(left)!=0 and len(right)!=0 and sum(left)/len(left)<sum(right)/len(right)):
                rospy.loginfo('turning right')
                return -1   # turn right
            else:
                rospy.loginfo('turning left')
                return 1    # turn left

        def uturn():
            lidar_distances = self.get_scan()
            uturn_counter = 0
            right = [x for x in lidar_distances[:45] if x != 10]
            left = [x for x in lidar_distances[45:] if x != 10]
            if (len(left)!=0 and len(right)!=0 and (sum(left)/len(left)+sum(right)/len(right))/2 < 4*LIDAR_ERROR):
                uturn_counter = 1
                updateVelocity(0.0, 1.5, 0, 0)
                rospy.loginfo("UTURN!")
                time.sleep(2)


        col = 0
        current_colour = "null"
        previous_colour = "null"
        victim = 0
        uturn_counter = 0
        soft_turns = 0
        center_avg = 0


        while not rospy.is_shutdown():
            previous_colour = current_colour
            current_colour = getAndUpdateColour()
            if (current_colour != previous_colour):
                if(current_colour == "red"):
                    victim += 1

                    for i in range(2):
                        GPIO.output(LED, GPIO.HIGH)
                        time.sleep(0.25)
                        GPIO.output(LED, GPIO.LOW)
                        time.sleep(0.25)

            if (col>2 and uturn_counter !=1):
                updateVelocity(0.0, 1.5, 0, 0)
                rospy.loginfo("Too many collisions, making uturn")
                time.sleep(1.5)
                uturn_counter = 1
                collision_counter -= 2

            lidar_distances = self.get_scan()
            min_distance = min(lidar_distances)
            center = [x for x in lidar_distances[36:54] if x != 10]
            if len(center) !=0:
                center_avg = sum(center)/len(center)
            rospy.loginfo(center_avg)

            # rospy.loginfo('Minimum distance to obstacle: %f', min_distance)
            uturn()

            if min_distance <= 2.5*LIDAR_ERROR or center_avg <= 4.5*LIDAR_ERROR:
                if turtlebot_moving:
                    # Determine direction to turn based on lidar data
                    speed_updates, speed_accumulation = updateVelocity(0.5, (0.9*direction()), speed_updates, speed_accumulation)

                    rospy.loginfo('Collision!')
                    collision_counter+=1
                    col+=1
                    for i in range(1):
                        GPIO.output(LED, GPIO.HIGH)
                        time.sleep(0.5)
                        GPIO.output(LED, GPIO.LOW)

                    time.sleep(0.1)
            elif 2.5*LIDAR_ERROR < min_distance < EMERGENCY_STOP_DISTANCE or 4.5*LIDAR_ERROR < center_avg < SAFE_STOP_DISTANCE:

                speed_updates, speed_accumulation = updateVelocity(0.7, (0.8*direction()), speed_updates, speed_accumulation)
                time.sleep(0.1)
                rospy.loginfo('Sharp turn')

            elif EMERGENCY_STOP_DISTANCE < min_distance < SAFE_STOP_DISTANCE or SAFE_STOP_DISTANCE < center_avg < 0.5 :
                speed_updates, speed_accumulation = updateVelocity(0.8, (0.5*direction()), speed_updates, speed_accumulation)
                time.sleep(0.1)
                col = 0
                rospy.loginfo('soft turn')
                soft_turns += 1

            else:
                speed_updates, speed_accumulation = updateVelocity(1, 0.0, speed_updates, speed_accumulation)
                turtlebot_moving = True
                # rospy.loginfo('Distance of the obstacle : %f', min_distance)
                col = 0
                uturn_counter = 0
                soft_turns = 0

            # Check if two minutes have elapsed
            elapsed_time = rospy.Time.now() - self.start_time
            if elapsed_time.to_sec() >= 120:                                    # Changed to 120 sec
                rospy.loginfo("Two minutes have passed. Stopping the robot.")   # Changed to 2 minuttes
                speed_updates, speed_accumulation = updateVelocity(0.0, 0.0, speed_updates, speed_accumulation)
                average_linear_speed = speed_accumulation/speed_updates
                rospy.loginfo('The average speed this round was: %f', average_linear_speed)
                rospy.loginfo("Total number of collisions was: %f", collision_counter)
                rospy.loginfo("The total number of victims detected: %f", victim)
                break


def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
