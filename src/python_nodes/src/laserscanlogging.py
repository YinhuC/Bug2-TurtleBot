#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan


class LaserScanOutput():
    def __init__(self):
        rospy.init_node("LaserScanOutput")
        rospy.Subscriber("/scan",
                         LaserScan,
                         self.OnLaserScan)
        rospy.spin()

    def OnLaserScan(self, data):
        count = len(data.ranges)
        # Assuming the middle point is directly in front of the robot
        rospy.loginfo("Front value is %f", data.ranges[count / 2])


if __name__ == '__main__':
    LaserScanOutput()
