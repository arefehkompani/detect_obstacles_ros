#!/usr/bin/python3

import rospy
import math
from calculate_distance.srv import GetDistance, GetDistanceResponse
from nav_msgs.msg import Odometry

class Distance_calculator:
    def __init__(self) -> None:
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
        
    def calculate_distance(self, obstacle):
        msg = rospy.wait_for_message("/odom", Odometry)
        position = msg.pose.pose.position
        distance = math.sqrt((abs(obstacle[1][0] - position.x)**2 + abs(obstacle[1][1] - position.y)**2))
        return distance

    def get_distance(self, req):
        ob_name = req.obstacle_name
        rospy.loginfo(f"NEW CALL: {ob_name}")
        distance = -1

        if ob_name in self.obstacles:
            distance = self.calculate_distance()
        else:
            rospy.logerr(f'ob_name is not valid: {ob_name}')
        
        res = GetDistanceResponse()
        res.distance = distance

        return res        
        
def listener():
    rospy.init_node('distance_node_calculator', anonymous=True)
    dc = Distance_calculator()
    rospy.Subscriber("/scan", LaserScan, dc.read_distance)
    s = rospy.service('/get_distance', GetDistance, dc.get_distance)
    rospy.spin()

if __name__=='__main__':
    listener()