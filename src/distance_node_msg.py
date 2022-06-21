#!/usr/bin/python3


import sys
import rospy
import math
from nav_msgs.msg import Odometry
from calculate_distance.msg import ClosestObstacle

class distance_node_msg:
    def __init__(self) -> None:
        rospy.init_node('distance_node_msg', anonymous=True)
        self.pub_distance = rospy.Publisher('/ClosestObstacle', ClosestObstacle, queue_size=10)

        self.obstacles = [
            ('bookshelf', (2.64, -1.55)),
            ('dumpster', (1.23, -4.57)),
            ('barrel', (-2.51, -3.08)),
            ('postbox', (-4.47, -0.57)),
            ('brick_box', (-3.44, 2.75)),
            ('cabinet', (-0.45, 4.05)),
            ('cafe_table', (1.91, 3.37)),
            ('fountain', (4.08, 1.14))
        ]

    def talk(self):
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            msg = ClosestObstacle()
            min_value = 1000
            
            for obstacle in self.obstacles:
                odom_msg = rospy.wait_for_message('/odom', Odometry)
                position  = odom_msg.pose.pose.position
                distance_temp = math.sqrt((abs(obstacle[1][0] - position.x)**2 + abs(obstacle[1][1] - position.y)**2))
                
                if distance_temp < min_value:
                    min_value = distance_temp
                    name = obstacle[0]

            msg.obstacle_name = name
            msg.distance = min_value

            rospy.loginfo(msg)
            self.pub_distance.publish(msg)

            rate.sleep()
            


if __name__ == '__main__':
    try:
        node = distance_node_msg()
        node.talk()
    except rospy.ROSInterruptException:
        pass
