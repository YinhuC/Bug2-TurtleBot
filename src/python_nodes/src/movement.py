#! /usr/bin/env python
import math
import rospy
from tf import transformations
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

ANGLE_ERROR_MARGIN = 0.1
DISTANCE_ERROR_MARGIN = 0.2
PI = 3.1415926535897


class MovementNode():

    # Scanner
    _regions = [
        0,
        0,
        0,
        0,
        0,
    ]

    # States
    GO_TO_GOAL = 1
    FOLLOW_WALL = 2
    WF_COMPLETE = 0

    # Substates
    FIX_HEADING = -10
    GO_FORWARD = 10
    END_STATE = 20

    # Robot state
    current_state = GO_TO_GOAL
    current_substate = FIX_HEADING
    current_position = Point()
    current_angle = 0
    setup = False

    # Destination point
    dest = Point()
    dest.x = 10
    dest.y = -1.6

    def __init__(self):
        # Initialise current node
        rospy.init_node('MovementNode', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher(
            '/cmd_vel_mux/input/teleop', Twist, queue_size=10)

        rospy.loginfo("To stop TurtleBot CTRL + C")

        # Subscribe to topics to get information from services
        rospy.Subscriber("/scan", LaserScan, self.OnLaserScanCallback)
        rospy.Subscriber("/odom", Odometry, self.OdometryCallback)

        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.current_substate == self.GO_TO_GOAL:
                self.GoToGoal(r)
            elif self.current_substate == self.FOLLOW_WALL:
                self.FollowWall(r)

    def FollowWall(self, r):
        if self.regions[1] < 0.8:
            self.StopTurn()
        elif self.current_substate == self.GO_FORWARD:
            self.GoForward()
        elif self.current_substate == self.END_STATE:
            self.EndState()
            pass
        r.sleep()

    def GoToGoal(self, r):
        if self.current_substate == self.FIX_HEADING:
            self.FixHeading()
        elif self.current_substate == self.GO_FORWARD:
            self.GoForward()
        elif self.current_substate == self.END_STATE:
            self.EndState()
            pass
        r.sleep()

    def FixHeading(self):
        desired_angle = math.atan2(
            self.dest.y - self.current_position.y, self.dest.x - self.current_position.x)
        angle_error = desired_angle - self.current_angle

        move_cmd = Twist()
        if math.fabs(angle_error) > ANGLE_ERROR_MARGIN:
            move_cmd.angular.z = 1 if angle_error > 0 else -1

        self.cmd_vel.publish(move_cmd)

        if math.fabs(angle_error) < ANGLE_ERROR_MARGIN:
            self.SwitchState(self.GO_FORWARD)

    def GoForward(self):
        desired_angle = math.atan2(
            self.dest.y - self.current_position.y, self.dest.x - self.current_position.x)
        angle_error = desired_angle - self.current_angle
        distance_error = math.sqrt(pow(self.dest.y - self.current_position.y, 2) +
                                   pow(self.dest.x - self.current_position.x, 2))

        if distance_error > DISTANCE_ERROR_MARGIN:
            move_cmd = Twist()
            move_cmd.linear.x = 0.6
            self.cmd_vel.publish(move_cmd)
        else:
            self.SwitchState(self.DEST)

        # state change conditions
        if math.fabs(angle_error) > ANGLE_ERROR_MARGIN:
            self.SwitchState(self.FIX_HEADING)

    def EndState(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0
        move_cmd.angular.z = 0
        self.cmd_vel.publish(move_cmd)

    def SwitchState(self, state):
        self.current_substate = state
        rospy.loginfo('State is now [%s]' % self.current_substate)

    def OnLaserScanCallback(self, data):
        count = len(data.ranges)

        self._regions = [
            data.ranges[0],
            data.ranges[count / 2],
        ]
        rospy.loginfo(self._regions)

    def OdometryCallback(self, data):
        # Get the position turtlebot2
        self.current_position = data.pose.pose.position

        # transform to radians
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.current_angle = euler[2]

    def shutdown(self):
        # Stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    MovementNode()
