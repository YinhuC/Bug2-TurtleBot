#! /usr/bin/env python
from __future__ import print_function
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import math
import rospy
from tf import transformations
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# Setup error margins and Pi
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

    image_received = False
    image = None
    image_count = 1

    # Destination point
    dest = Point()
    # dest.x = 10
    # dest.y = 0

    def __init__(self):
        # Initialise current node
        rospy.init_node('MovementNode', anonymous=False)
        rospy.on_shutdown(self.Shutdown)
        self.cmd_vel = rospy.Publisher(
            '/cmd_vel_mux/input/teleop', Twist, queue_size=10)

        rospy.loginfo("To stop TurtleBot CTRL + C")

        # Subscribe to topics to get information from services
        rospy.Subscriber("/camera/scan", LaserScan, self.OnLaserScanCallback)
        rospy.Subscriber("/odom", Odometry, self.OdometryCallback)

        # Setup photo and subscribe topic
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.PhotoCallback)
        rospy.sleep(1)

        # Report to ROS 30 times per second
        r = rospy.Rate(30)

        # Loop until it has reached destination
        while not rospy.is_shutdown():
            if self.current_state == self.GO_TO_GOAL:
                # Enter a state to follow a straight line
                self.GoToGoal(r)

            elif self.current_state == self.FOLLOW_WALL:
                # Enter state to follow object or wall
                self.FollowWall(r)

            else:
                pass

    # This method is a state that involves while loops of sub states to follow a wall
    # Inputs: self, and the rate r
    # Outputs: None
    def FollowWall(self, r):
        move_cmd = Twist()

        # This loop puts the robot into forward linear movement only
        while (self.current_state == self.FOLLOW_WALL) and (self.current_substate_wall == self.FORWARD):
            # rospy.loginfo("FORWARD")

            move_cmd.linear.x = 0.5
            move_cmd.angular.z = 0.0

            # Publish movement commands
            self.cmd_vel.publish(move_cmd)
            r.sleep()

            # Check if turtlebot should be in another sub state or state
            self.CheckGoToGoalState()

        # This loop puts the robot into positive angular movement only to turn left
        while (self.current_state == self.FOLLOW_WALL) and (self.current_substate_wall == self.TURN_LEFT):
            # rospy.loginfo("TURN_LEFT")

            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.5

            self.cmd_vel.publish(move_cmd)
            r.sleep()

            # Check if turtlebot should be in another sub state or state
            self.CheckGoToGoalState()

        # This loop puts the robot into negative angular movement only to turn right
        while (self.current_state == self.FOLLOW_WALL) and (self.current_substate_wall == self.TURN_RIGHT):
            # rospy.loginfo("TURN_RIGHT")

            move_cmd.linear.x = 0.0
            move_cmd.angular.z = -0.5

            self.cmd_vel.publish(move_cmd)
            r.sleep()

            # Check if turtlebot should be in another sub state or state
            self.CheckGoToGoalState()

        # This loop puts the robot into negative linear movement only, (is not used), however,
        # may be useful in future work and for completeness
        while (self.current_state == self.FOLLOW_WALL) and (self.current_substate_wall == self.GO_BACK):
            # rospy.loginfo("GO_BACK")

            move_cmd.linear.x = -0.2
            move_cmd.angular.z = 0.0

            self.cmd_vel.publish(move_cmd)
            r.sleep()

            # Check if turtlebot should be in another sub state or state
            self.CheckGoToGoalState()

        # This loop allows the robot to turn tight corners when it loses both readings of the wall
        while (self.current_state == self.FOLLOW_WALL) and (self.current_substate_wall == self.CORNER):
            # rospy.loginfo("CORNER")

            # Setup sub state of sub state and save time for calcualtions
            action = 0
            timeStart = rospy.Time.now().to_sec()

            # This loop moves the bot a certain distance past the wall, so after the turn it won't crash into the wall
            while (action == 0) and (self.current_state == self.FOLLOW_WALL):

                move_cmd.linear.x = 0.5
                move_cmd.angular.z = 0.0

                self.cmd_vel.publish(move_cmd)
                r.sleep()

                # Calculate the relative distance moved
                timeNow = rospy.Time.now().to_sec()
                distance_relative = 0.5*(timeNow-timeStart)

                MovementNode.last_dist = distance_relative

                # Check if turtlebot should be in another sub state or state
                self.CheckGoToGoalState()

                # Break loop if relative distance is reached
                if(distance_relative > 1.15):
                    action = 1

            timeStart = rospy.Time.now().to_sec()
            while (action == 1) and (self.current_state == self.FOLLOW_WALL):
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = -0.5

                self.cmd_vel.publish(move_cmd)
                r.sleep()

                # Calculate the relative angle turned
                timeNow = rospy.Time.now().to_sec()
                angle_relative = (0.5)*(timeNow-timeStart)

                # Check if turtlebot should be in another sub state or state
                self.CheckGoToGoalState()

                # Break loop after bot has turned 90 degrees
                if(angle_relative > 90*2*PI/360):
                    action = 2

            # Change state to move forward after it has turned the corner
            self.current_substate_wall = self.FORWARD

    # This method checks and compares the turtlebots readings to the thresholds to keep the bot following the wall
    def CheckGoToGoalState(self):

        # Change sub state to move forward if within thresholds
        if(math.isinf(self._regions[1]) or self._regions[1] > 0.6161) and (self._regions[0] < 1.05) \
                and (self._regions[0] > 0.85) and (self.current_substate_wall != self.FORWARD):
            self.current_substate_wall = self.FORWARD

        # Change sub state to turn left if below lower threshold (will crash into wall on current trajectory)
        elif(math.isinf(self._regions[1]) or self._regions[1] > 0.6161) and (self._regions[0] < 0.85) \
                and (self.current_substate_wall != self.TURN_LEFT):
            self.current_substate_wall = self.TURN_LEFT

        # Change sub state to turn right if above upper threshold (will lose readings of wall on current trajectory)
        elif(math.isinf(self._regions[1]) or self._regions[1] > 0.6161) and (self._regions[0] > 1.05) \
                and (self.current_substate_wall != self.TURN_RIGHT):
            self.current_substate_wall = self.TURN_RIGHT

        # Change sub state to turn a corner if both readings are above a certain threshold
        elif((math.isinf(self._regions[1]) or (self._regions[1] > 0.6161)) and (math.isinf(self._regions[0]) or (self._regions[0] > 1.5))):
            self.current_substate_wall = self.CORNER
        else:
            pass

        # Change state to go towards goal instead of following wall as bot has returned to original path
        if (self.current_position.y > self.past_side_state - 0.05) and (self.current_position.y < self.past_side_state + 0.05) \
                and (self.current_position.x - self.past_trajectory_state > 0.7):
            self.current_state = self.GO_TO_GOAL
            rospy.loginfo("TurtleBot now going to goal")

    # This method directs the bot towards the goal and consists of sub states
    def GoToGoal(self, r):

        # Check if there is a wall to change state and avoid collision
        if (self._regions[1] < 0.6161) and not math.isinf(self._regions[1]):
            # Setup sub states
            self.current_state = self.FOLLOW_WALL
            self.current_substate_wall = self.TURN_LEFT

            # Save last position not folloiwing wall into memory
            self.past_side_state = self.current_position.y
            self.past_trajectory_state = self.current_position.x

            # Take photo and save to current working directory
            title = rospy.get_param(
                '~image_title', "object_" + str(self.image_count) + ".jpg")
            if self.TakePicture(title):
                rospy.loginfo(
                    "Image saved to current working directory: " + title)
                self.image_count += 1
            else:
                rospy.loginfo("No images received or saved")

            rospy.loginfo("TurtleBot now following wall")

        elif self.current_substate_goal == self.FIX_HEADING:
            # Fix heading due to angle error
            self.FixHeading()
        elif self.current_substate_goal == self.GO_FORWARD:
            # Move towards goal as bot is on correct trajectory
            self.GoForward()
        elif self.current_substate_goal == self.END_STATE:
            self.EndState()
        else:
            rospy.logerr('Unkown state for GO_TO_GOAL')
            pass
        r.sleep()

    # This method fixes the heading/direction of the bot if it has gone off the original trajectory
    # which is determined by the angle error threshold
    def FixHeading(self):

        # Calculate angle error
        angle_error = self.CalculateAngleError()

        # Check which direction bot is moving, North or South, and use those movements
        # The bot was designed to start at (0,0) and move towards a positive x direction
        # If current angle error is greater than threshold then adjust course
        move_cmd = Twist()
        if ((math.fabs(angle_error) > ANGLE_ERROR_MARGIN) and ((self.dest.x - self.current_position.x) >= 0)):
            move_cmd.angular.z = 0.8 if angle_error > 0 else -0.8
        elif ((math.fabs(angle_error) > ANGLE_ERROR_MARGIN) and ((self.dest.x - self.current_position.x) < 0)):
            move_cmd.angular.z = -0.8 if angle_error > 0 else 0.8
        else:
            pass

        self.cmd_vel.publish(move_cmd)

        # If angle error is smaller than the threshold then change sub state to move forward
        if math.fabs(angle_error) < ANGLE_ERROR_MARGIN:
            self.current_substate_goal = self.GO_FORWARD
            rospy.loginfo("GO_FORWARD")

    # This method move the robot towards the goal
    def GoForward(self):

        # Calculate angle error
        angle_error = self.CalculateAngleError()

        # Calculate distance error
        distance_error = math.sqrt(pow(self.dest.y - self.current_position.y, 2) +
                                   pow(self.dest.x - self.current_position.x, 2))

        # If current angle error is larger than threshold, then change sub state to fix heading
        # If distance error is larger than threhsold then move towards goal, else bot has arrived at destination
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

    # This method calculates the angle error
    def CalculateAngleError(self):
        desired_angle = math.atan2(
            self.dest.y - self.current_position.y, self.dest.x - self.current_position.x)
        return desired_angle - self.current_angle

    # This method is the callback method for the pointcloud to laserscan reader
    def OnLaserScanCallback(self, data):

        # Setup reading values by taking the smallest value on the rightside (going over a little to the left side)
        # and take the smallest value in front to avoid wall/object collision
        self._regions = [
            min(data.ranges[0:369]),
            min(data.ranges[139:499]),
        ]
        # rospy.loginfo(self._regions)

    # This method takes a photo and saves it to the current working directory
    def TakePicture(self, title):
        if self.image_received:
            # Save an image
            cv2.imwrite(title, self.image)
            return True
        else:
            return False

    # This is the call back to convert the image to OpenCV format
    def PhotoCallback(self, data):
        # Convert image to OpenCV format else print error
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Setup variables for image to be saved
        self.image_received = True
        self.image = cv_image

    # This method is the callback method for the odometry
    def OdometryCallback(self, data):
        # Get the position turtlebot2
        self.current_position = data.pose.pose.position

        # Setup poistion information: bot was implemented to move towards a positive x direction (One of the assumptions made or
        # else there would be too many options) Marker can change direction: Add or substract from x and y to change goal direction
        if (self.setup == False):
            # Aim to get to other side of room
            self.dest.x = self.current_position.x + 20
            self.dest.y = self.current_position.y
            self.dest.z = 0
            self.setup = True

        # Transform information and use euler to get the current angle of the bot for calculations later
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.current_angle = euler[2]

    # This method logs info that the bot has been terminated
    def Shutdown(self):
        rospy.loginfo("Stop TurtleBot")
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)


if __name__ == '__main__':
    MovementNode()
