#! /usr/bin/env python
import math
import rospy
from tf import transformations
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

ANGLE_ERROR_MARGIN = 0.06
DISTANCE_ERROR_MARGIN = 0.1
PI = 3.1415926535897


class MovementNode():

    # Scanner
    _regions = [
        5,
        5
    ]

    # States
    GO_TO_GOAL = 1
    FOLLOW_WALL = 2
    WF_COMPLETE = 0

    # Substates
    FIX_HEADING = -10
    GO_FORWARD = 11
    END_STATE = 12

    GO_BACK = 20
    FORWARD = 21
    TURN_LEFT = 22
    TURN_RIGHT = 23
    CORNER = 24

    # Robot state
    current_state = GO_TO_GOAL
    current_substate_goal = FIX_HEADING
    current_substate_wall = TURN_LEFT
    current_position = Point()
    current_angle = 0
    setup = False

    # Destination point
    dest = Point()
    dest.x = 20
    dest.y = 0

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

        r = rospy.Rate(30)

        while not rospy.is_shutdown():
            if self.current_state == self.GO_TO_GOAL:
                self.GoToGoal(r)
            elif self.current_state == self.FOLLOW_WALL:
                self.FollowWall(r)
            else:
                pass

    def FollowWall(self, r):
        move_cmd = Twist()

        while self.current_substate_wall == self.FORWARD:
            rospy.loginfo("TurtleBot forward")

            move_cmd.linear.x = 0.5
            move_cmd.angular.z = 0.0

            self.cmd_vel.publish(move_cmd)
            r.sleep()

            self.CheckChangeState()

        while self.current_substate_wall == self.TURN_LEFT:
            rospy.loginfo("TurtleBot left")

            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.5

            self.cmd_vel.publish(move_cmd)
            r.sleep()

            self.CheckChangeState()

        while self.current_substate_wall == self.TURN_RIGHT:
            rospy.loginfo("TurtleBot right")

            move_cmd.linear.x = 0.0
            move_cmd.angular.z = -0.5

            self.cmd_vel.publish(move_cmd)
            r.sleep()

            self.CheckChangeState()

        while self.current_substate_wall == self.GO_BACK:
            rospy.loginfo("TurtleBot back")

            move_cmd.linear.x = -0.2
            move_cmd.angular.z = 0.0

            self.cmd_vel.publish(move_cmd)
            r.sleep()

            self.CheckChangeState()

        while self.current_substate_wall == self.CORNER:
            action = 0
            timeStart = rospy.Time.now().to_sec()
            while action == 0:
                rospy.loginfo("C 1")

                move_cmd.linear.x = 0.5
                move_cmd.angular.z = 0.0

                self.cmd_vel.publish(move_cmd)
                r.sleep()

                timeNow = rospy.Time.now().to_sec()
                distance_relative = 0.5*(timeNow-timeStart)

                MovementNode.last_dist = distance_relative

                if(distance_relative > 1.15):
                    action = 1

            timeStart = rospy.Time.now().to_sec()
            while action == 1:
                rospy.loginfo("C 2")
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = -0.5

                self.cmd_vel.publish(move_cmd)
                r.sleep()

                timeNow = rospy.Time.now().to_sec()
                angle_relative = (0.5)*(timeNow-timeStart)

                if(angle_relative > 90*2*PI/360):
                    action = 2

            self.current_substate_wall = self.FORWARD

    def CheckChangeState(self):
        if(math.isnan(self._regions[1]) or self._regions[1] > 0.6161) and (self._regions[0] < 1.05) \
                and (self._regions[0] > 0.85) and (self.current_substate_wall != self.FORWARD):
            self.current_substate_wall = self.FORWARD
        elif(math.isnan(self._regions[1]) or self._regions[1] > 0.6161) and (self._regions[0] < 0.85) \
                and (self.current_substate_wall != self.TURN_LEFT):
            self.current_substate_wall = self.TURN_LEFT
        elif(math.isnan(self._regions[1]) or self._regions[1] > 0.6161) and (self._regions[0] > 1.05) \
                and (self.current_substate_wall != self.TURN_RIGHT):
            self.current_substate_wall = self.TURN_RIGHT
        elif(math.isnan(self._regions[1])) and (math.isnan(self._regions[0])):
            self.current_substate_wall = self.CORNER
        else:
            pass

    def GoToGoal(self, r):

        if (self._regions[1] < 0.6161) and not math.isnan(self._regions[1]):
            self.current_state = self.FOLLOW_WALL
            rospy.loginfo("TurtleBot now following wall")
        elif self.current_substate_goal == self.FIX_HEADING:
            self.FixHeading()
        elif self.current_substate_goal == self.GO_FORWARD:
            self.GoForward()
        elif self.current_substate_goal == self.END_STATE:
            self.EndState()
        else:
            rospy.logerr('Unkown state for GO_TO_GOAL')
            pass
        r.sleep()

    def FixHeading(self):
        desired_angle = math.atan2(
            self.dest.y - self.current_position.y, self.dest.x - self.current_position.x)
        angle_error = desired_angle - self.current_angle

        move_cmd = Twist()
        if math.fabs(angle_error) > ANGLE_ERROR_MARGIN:
            move_cmd.angular.z = 0.8 if angle_error > 0 else -0.8

        self.cmd_vel.publish(move_cmd)

        if math.fabs(angle_error) < ANGLE_ERROR_MARGIN:
            self.current_substate_goal = self.GO_FORWARD

    def GoForward(self):
        desired_angle = math.atan2(
            self.dest.y - self.current_position.y, self.dest.x - self.current_position.x)
        angle_error = desired_angle - self.current_angle
        distance_error = math.sqrt(pow(self.dest.y - self.current_position.y, 2) +
                                   pow(self.dest.x - self.current_position.x, 2))

        move_cmd = Twist()
        if math.fabs(angle_error) > ANGLE_ERROR_MARGIN:
            self.current_substate_goal = self.FIX_HEADING
        elif distance_error > DISTANCE_ERROR_MARGIN:
            move_cmd.linear.x = 0.5
            self.cmd_vel.publish(move_cmd)
        else:
            move_cmd.linear.x = 0.0
            self.cmd_vel.publish(move_cmd)

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
