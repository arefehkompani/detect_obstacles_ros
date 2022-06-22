#!/usr/bin/python3

import rospy
import math
from calculate_distance.srv import GetDistance, GetDistanceResponse
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

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
        
    def calculate_distance(self, index):
        msg = rospy.wait_for_message("/odom", Odometry)
        position = msg.pose.pose.position

        obstacle = self.obstacles[index]
        obstacle_name, obstacle_pos = obstacle
        distance = math.sqrt((abs(obstacle_pos[0] - position.x)**2 + abs(obstacle_pos[1] - position.y)**2))
        rospy.loginfo(distance)
        return distance

    def get_distance(self, req):
        ob_name = req.obstacle_name
        rospy.loginfo(f"NEW CALL: {ob_name}")
        distance = -1

        obstacle_names = list(map(lambda x : x[0], self.obstacles))

        if ob_name in obstacle_names:
            index = obstacle_names.index(ob_name)
            rospy.loginfo("here")
            distance = self.calculate_distance(index)
        else:
            rospy.logerr(f'ob_name is not valid: {ob_name}')
        
        res = GetDistanceResponse()
        res.distance = distance
        return res        
        
def listener():
    rospy.init_node('distance_node_calculator', anonymous=True)
    dc = Distance_calculator()
    #rospy.Subscriber("/scan", LaserScan, dc.read_distance)
    s = rospy.Service('/get_distance', GetDistance, dc.get_distance)
    rospy.spin()

if __name__=='__main__':
    listener()
