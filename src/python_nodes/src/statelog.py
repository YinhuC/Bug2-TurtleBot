#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class StateLog():

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

    def __init__(self):
        rospy.init_node("StateLogNode")

        # Subscribe to topics
        rospy.Subscriber("/camera/scan", LaserScan, self.LaserScanCallback)
        rospy.Subscriber("/cmd_vel_mux/input/teleop",
                         Twist, self.MovementCallback)

        # 1 second to boot all nodes
        rospy.sleep(1)

        rospy.loginfo("StateLogNode has initialised")

        # Keep node running
        rospy.spin()

    # This method is the callback method for the pointcloud to laserscan reader
    def LaserScanCallback(self, data):

        # Setup reading values by taking the smallest value on the rightside (going over a little to the left side)
        # and take the smallest value in front to avoid wall/object collision
        self._regions = [
            min(data.ranges[0:369]),
            min(data.ranges[139:499]),
        ]

        # For testing
        # rospy.loginfo(self._regions)

    def MovementCallback(self, data):
        self.last_x = data.linear.x
        self.last_z = data.angular.z

        self.AssignState()
        self.LogStateChange()

    # Log state change if state has changed
    def LogStateChange(self):

        if (self.last_state != self.current_state) and (self.current_state == self.GO_TO_GOAL):
            rospy.loginfo("Bot changed state: GO_TO_GOAL")

        elif(self.last_state != self.current_state) and (self.current_state == self.FOLLOW_WALL):
            rospy.loginfo("Bot changed state: FOLLOW_WALL")

        else:
            pass

        if (self.last_substate_goal != self.current_substate_goal) and (self.current_substate_goal == self.FIX_HEADING):
            rospy.loginfo("Bot changed GO_TO_GOAL sub state: FIX_HEADING")

        elif(self.last_substate_goal != self.current_substate_goal) and (self.current_substate_goal == self.GO_FORWARD):
            rospy.loginfo("Bot changed GO_TO_GOAL sub state: GO_FORWARD")

        elif(self.last_substate_goal != self.current_substate_goal) and (self.current_substate_goal == self.END_STATE):
            rospy.loginfo("Bot changed GO_TO_GOAL sub state: END_STATE")

        else:
            pass

        if (self.last_substate_wall != self.current_substate_wall) and (self.current_substate_wall == self.FORWARD):
            rospy.loginfo("Bot changed FOLLOW_WALL sub state: FORWARD")

        elif(self.last_substate_wall != self.current_substate_wall) and (self.current_substate_wall == self.TURN_LEFT):
            rospy.loginfo("Bot changed FOLLOW_WALL sub state: TURN_LEFT")

        elif(self.last_substate_wall != self.current_substate_wall) and (self.current_substate_wall == self.TURN_RIGHT):
            rospy.loginfo("Bot changed FOLLOW_WALL sub state: TURN_RIGHT")

        elif(self.last_substate_wall != self.current_substate_wall) and (self.current_substate_wall == self.CORNER):
            rospy.loginfo("Bot changed FOLLOW_WALL sub state: CORNER")

        else:
            pass

    # Change state changes according to movements
    def AssignState(self):
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


if __name__ == '__main__':
    StateLog()
