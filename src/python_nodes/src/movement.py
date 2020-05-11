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

    past_side_state = 0
    past_trajectory_state = 0

    # Destination point
    dest = Point()
    # dest.x = 10
    # dest.y = 0

    def __init__(self):
        # Initialise current node
        rospy.init_node('MovementNode', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher(
            '/cmd_vel_mux/input/teleop', Twist, queue_size=10)

        rospy.loginfo("To stop TurtleBot CTRL + C")

        # Subscribe to topics to get information from services
        rospy.Subscriber("/camera/scan", LaserScan, self.OnLaserScanCallback)
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

        while (self.current_state == self.FOLLOW_WALL) and (self.current_substate_wall == self.FORWARD):
            rospy.loginfo("FORWARD")

            move_cmd.linear.x = 0.5
            move_cmd.angular.z = 0.0

            self.cmd_vel.publish(move_cmd)
            r.sleep()

            self.CheckGoToGoalState()

        while (self.current_state == self.FOLLOW_WALL) and (self.current_substate_wall == self.TURN_LEFT):
            rospy.loginfo("TURN_LEFT")

            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.5

            self.cmd_vel.publish(move_cmd)
            r.sleep()

            self.CheckGoToGoalState()

        while (self.current_state == self.FOLLOW_WALL) and (self.current_substate_wall == self.TURN_RIGHT):
            rospy.loginfo("TURN_RIGHT")

            move_cmd.linear.x = 0.0
            move_cmd.angular.z = -0.5

            self.cmd_vel.publish(move_cmd)
            r.sleep()

            self.CheckGoToGoalState()

        while (self.current_state == self.FOLLOW_WALL) and (self.current_substate_wall == self.GO_BACK):
            rospy.loginfo("GO_BACK")

            move_cmd.linear.x = -0.2
            move_cmd.angular.z = 0.0

            self.cmd_vel.publish(move_cmd)
            r.sleep()

            self.CheckGoToGoalState()

        while (self.current_state == self.FOLLOW_WALL) and (self.current_substate_wall == self.CORNER):
            rospy.loginfo("CORNER")

            action = 0
            timeStart = rospy.Time.now().to_sec()
            while (action == 0) and (self.current_state == self.FOLLOW_WALL):

                move_cmd.linear.x = 0.5
                move_cmd.angular.z = 0.0

                self.cmd_vel.publish(move_cmd)
                r.sleep()

                timeNow = rospy.Time.now().to_sec()
                distance_relative = 0.5*(timeNow-timeStart)

                MovementNode.last_dist = distance_relative

                self.CheckGoToGoalState()

                if(distance_relative > 1.15):
                    action = 1

            timeStart = rospy.Time.now().to_sec()
            while (action == 1) and (self.current_state == self.FOLLOW_WALL):
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = -0.5

                self.cmd_vel.publish(move_cmd)
                r.sleep()

                timeNow = rospy.Time.now().to_sec()
                angle_relative = (0.5)*(timeNow-timeStart)

                self.CheckGoToGoalState()

                if(angle_relative > 90*2*PI/360):
                    action = 2

            self.current_substate_wall = self.FORWARD

    def CheckGoToGoalState(self):
        if(math.isinf(self._regions[1]) or self._regions[1] > 0.6161) and (self._regions[0] < 1.05) \
                and (self._regions[0] > 0.85) and (self.current_substate_wall != self.FORWARD):
            self.current_substate_wall = self.FORWARD
        elif(math.isinf(self._regions[1]) or self._regions[1] > 0.6161) and (self._regions[0] < 0.85) \
                and (self.current_substate_wall != self.TURN_LEFT):
            self.current_substate_wall = self.TURN_LEFT
        elif(math.isinf(self._regions[1]) or self._regions[1] > 0.6161) and (self._regions[0] > 1.05) \
                and (self.current_substate_wall != self.TURN_RIGHT):
            self.current_substate_wall = self.TURN_RIGHT
        elif((math.isinf(self._regions[1]) or (self._regions[1] > 0.6161)) and (math.isinf(self._regions[0]) or (self._regions[0] > 1.5))):
            self.current_substate_wall = self.CORNER
        else:
            pass

        if (self.current_position.y > self.past_side_state - 0.05) and (self.current_position.y < self.past_side_state + 0.05) \
                and (self.current_position.x - self.past_trajectory_state > 0.7):
            self.current_state = self.GO_TO_GOAL
            rospy.loginfo("TurtleBot now going to goal")

    def GoToGoal(self, r):

        if (self._regions[1] < 0.6161) and not math.isinf(self._regions[1]):
            self.current_state = self.FOLLOW_WALL
            self.current_substate_wall = self.TURN_LEFT
            rospy.loginfo("TurtleBot now following wall")
            self.past_side_state = self.current_position.y
            self.past_trajectory_state = self.current_position.x
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
        angle_error = self.CalculateAngleError()
        move_cmd = Twist()

        if ((math.fabs(angle_error) > ANGLE_ERROR_MARGIN) and ((self.dest.x - self.current_position.x) >= 0)):
            move_cmd.angular.z = 0.8 if angle_error > 0 else -0.8
        elif ((math.fabs(angle_error) > ANGLE_ERROR_MARGIN) and ((self.dest.x - self.current_position.x) < 0)):
            move_cmd.angular.z = -0.8 if angle_error > 0 else 0.8
        else:
            pass

        self.cmd_vel.publish(move_cmd)

        if math.fabs(angle_error) < ANGLE_ERROR_MARGIN:
            self.current_substate_goal = self.GO_FORWARD
            rospy.loginfo("GO_FORWARD")

    def GoForward(self):
        angle_error = self.CalculateAngleError()
        distance_error = math.sqrt(pow(self.dest.y - self.current_position.y, 2) +
                                   pow(self.dest.x - self.current_position.x, 2))
        move_cmd = Twist()
        if math.fabs(angle_error) > ANGLE_ERROR_MARGIN:
            self.current_substate_goal = self.FIX_HEADING
            rospy.loginfo("FIX_HEADING")
        elif distance_error > DISTANCE_ERROR_MARGIN:
            move_cmd.linear.x = 0.5
            self.cmd_vel.publish(move_cmd)
        else:
            move_cmd.linear.x = 0.0
            self.cmd_vel.publish(move_cmd)

    def CalculateAngleError(self):
        desired_angle = math.atan2(
            self.dest.y - self.current_position.y, self.dest.x - self.current_position.x)
        return desired_angle - self.current_angle

    def OnLaserScanCallback(self, data):
        count = len(data.ranges)

        self._regions = [
            min(data.ranges[0:369]),
            min(data.ranges[139:499]),
        ]
        # rospy.loginfo(self._regions)

    def OdometryCallback(self, data):
        # Get the position turtlebot2
        self.current_position = data.pose.pose.position

        if (self.setup == False):
            # Aim to get to other side of room
            self.dest.x = self.current_position.x + 20
            self.dest.y = self.current_position.y
            self.dest.z = 0
            self.setup = True

        # transform to radians
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.current_angle = euler[2]

        # rospy.loginfo(self.current_angle)

    def shutdown(self):
        # Stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    MovementNode()
