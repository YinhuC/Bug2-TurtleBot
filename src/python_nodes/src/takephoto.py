#! /usr/bin/env python
from __future__ import print_function
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist


class TakePhoto:

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
    last_state = GO_TO_GOAL
    current_state = GO_TO_GOAL

    last_substate_goal = FIX_HEADING
    current_substate_goal = FIX_HEADING

    last_substate_wall = TURN_LEFT
    current_substate_wall = TURN_LEFT

    last_x = 0
    last_z = 0

    image_received = False
    image = None
    image_count = 1

    def __init__(self):
        rospy.init_node("PhotoTakeNode")

        # Setup photo and subscribe topic
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.PhotoCallback)
        rospy.Subscriber("/cmd_vel_mux/input/teleop",
                         Twist, self.MovementCallback)

        # 1 second to boot all nodes
        rospy.sleep(1)

        rospy.loginfo("PhotoTakeNode has initialised")

        # Keep node running
        rospy.spin()

    # Log state change if state has changed
    def CheckState(self):

        # If object detected
        if(self.last_state != self.current_state) and (self.current_state == self.FOLLOW_WALL):

            # Take photo and save to current working directory, then increment photo number
            title = rospy.get_param(
                '~image_title', "object_" + str(self.image_count) + ".jpg")
            if self.TakePicture(title):
                rospy.loginfo(
                    "Object encountered - Image saved to current working directory as : " + title)
                self.image_count += 1
            else:
                rospy.loginfo("No images received or saved")

    # This method takes a photo and saves it to the current working directory
    def TakePicture(self, title):
        if self.image_received:
            # Save an image
            cv2.imwrite(title, self.image)
            return True
        else:
            return False

    # Change state changes according to movements
    def AssignState(self):

        # Compare movements to determine which state. Save current state as last state,
        # then save new state as current state for both sub states and states
        if (self.last_x == 0.6) or (abs(self.last_z) == 0.8):
            self.last_state = self.current_state
            self.current_state = self.GO_TO_GOAL

            if (self.last_x == 0.0) and (abs(self.last_z) == 0.8):
                self.last_substate_goal = self.current_substate_goal
                self.current_substate_goal = self.FIX_HEADING

            elif (self.last_z == 0.0) and (self.last_x == 0.6):
                self.last_substate_goal = self.current_substate_goal
                self.current_substate_goal = self.GO_FORWARD

            else:
                pass

        elif ((self.last_x == 0.5) or (self.last_x == 0.4)) or ((abs(self.last_z) == 0.5) or (abs(self.last_z) == 0.4)):
            self.last_state = self.current_state
            self.current_state = self.FOLLOW_WALL

            if (self.last_x == 0.5) and (self.last_z == 0.0):
                self.last_substate_wall = self.current_substate_wall
                self.current_substate_wall = self.FORWARD

            elif (self.last_x == 0.0) and (self.last_z == 0.5):
                self.last_substate_wall = self.current_substate_wall
                self.current_substate_wall = self.TURN_LEFT

            elif (self.last_x == 0.0) and (self.last_z == -0.5):
                self.last_substate_wall = self.current_substate_wall
                self.current_substate_wall = self.TURN_RIGHT

            elif ((self.last_x == 0.4) and (self.last_z == 0.0)) or (self.last_x == 0.0) and (self.last_z == -0.4):
                self.last_substate_wall = self.current_substate_wall
                self.current_substate_wall = self.CORNER

            else:
                pass

        else:
            pass

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

    # Save movement infromation and then do checks/take picture
    def MovementCallback(self, data):
        self.last_x = data.linear.x
        self.last_z = data.angular.z

        self.AssignState()
        self.CheckState()


if __name__ == '__main__':
    TakePhoto()
