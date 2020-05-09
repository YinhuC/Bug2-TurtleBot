#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent  # Import the bumper message

PI = 3.1415926535897
perpendicular = 90*2*PI/360


class MovementNode():

    # Define some state constants so we know what is happening
    MOVING_FORWARD = 0
    MOVING_BACKWARDS = 1

    LEFT = 2
    RIGHT = 3
    FORWARD_TIMED = 4

    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3

    # Current state of the robot
    state = MOVING_FORWARD
    direction = NORTH

    hit = False
    corner_count = 0
    sideways_counter = 0

    last_dist = 0
    desired_dist = 0

    def __init__(self):
        # Initialise everything - see the lab manual for descriptions
        rospy.init_node('MovementNode', anonymous=False)
        rospy.loginfo("To stop TurtleBot CTRL + C")
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher(
            '/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        r = rospy.Rate(10)

        # Need to listen to the bumper sensor
        rospy.Subscriber("/mobile_base/events/bumper",
                         BumperEvent, self.OnBumperCallback)

        # Define the standard movement (forward at 0.2 m/s)
        move_cmd = Twist()

        angle_relative = 0
        distance_relative = 0

        # As long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():

            rospy.loginfo(MovementNode.direction)
            rospy.loginfo(MovementNode.sideways_counter)

            while MovementNode.state == MovementNode.MOVING_FORWARD:
                move_cmd.linear.x = 0.2
                move_cmd.angular.z = 0.0

                self.cmd_vel.publish(move_cmd)
                r.sleep()

            MovementNode.forward(self, r, move_cmd)

            MovementNode.back(self, r, move_cmd)

            if(MovementNode.sideways_counter > 0):
                MovementNode.left(self, r, move_cmd)

            if(MovementNode.sideways_counter < 0):
                MovementNode.right(self, r, move_cmd)

    def forward(self, r, move_cmd):
        timeStart = rospy.Time.now().to_sec()
        while MovementNode.state == MovementNode.FORWARD_TIMED:
            move_cmd.linear.x = 0.2
            move_cmd.angular.z = 0.0

            self.cmd_vel.publish(move_cmd)
            r.sleep()

            timeNow = rospy.Time.now().to_sec()
            distance_relative = 0.2*(timeNow-timeStart)

            MovementNode.last_dist = distance_relative

            if(distance_relative > 1):
                MovementNode.state = MovementNode.LEFT

    def back(self, r, move_cmd):
        timeStart = rospy.Time.now().to_sec()
        while MovementNode.state == MovementNode.MOVING_BACKWARDS:
            move_cmd.linear.x = -0.2
            move_cmd.angular.z = 0.0

            self.cmd_vel.publish(move_cmd)
            r.sleep()

            timeNow = rospy.Time.now().to_sec()
            distance_relative = 0.2*(timeNow-timeStart)

            if(distance_relative > 0.2):
                if(MovementNode.direction == MovementNode.NORTH):
                    MovementNode.state = MovementNode.LEFT
                elif(MovementNode.direction == MovementNode.WEST):
                    MovementNode.state = MovementNode.RIGHT
                elif(MovementNode.direction == MovementNode.EAST):
                    MovementNode.state = MovementNode.LEFT

    def left(self, r, move_cmd):
        timeStart = rospy.Time.now().to_sec()
        while MovementNode.state == MovementNode.LEFT:

            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 1.8

            self.cmd_vel.publish(move_cmd)
            r.sleep()

            timeNow = rospy.Time.now().to_sec()
            angle_relative = (1.8)*(timeNow-timeStart)

            if(angle_relative > perpendicular):
                if(MovementNode.sideways_counter == 0):
                    MovementNode.state = MovementNode.MOVING_FORWARD
                else:
                    MovementNode.state = MovementNode.FORWARD_TIMED

                if (MovementNode.direction == MovementNode.NORTH):
                    MovementNode.direction = MovementNode.WEST
                elif (MovementNode.direction == MovementNode.WEST):
                    MovementNode.direction = MovementNode.SOUTH
                elif (MovementNode.direction == MovementNode.SOUTH):
                    MovementNode.direction = MovementNode.EAST
                elif (MovementNode.direction == MovementNode.EAST):
                    MovementNode.direction = MovementNode.NORTH

    def right(self, r, move_cmd):
        timeStart = rospy.Time.now().to_sec()
        while MovementNode.state == MovementNode.RIGHT:

            move_cmd.linear.x = 0.0
            move_cmd.angular.z = -1.8

            self.cmd_vel.publish(move_cmd)
            r.sleep()

            timeNow = rospy.Time.now().to_sec()
            angle_relative = (1.8)*(timeNow-timeStart)

            if(angle_relative > perpendicular):
                if(MovementNode.sideways_counter == 0):
                    MovementNode.state = MovementNode.MOVING_FORWARD
                else:
                    MovementNode.state = MovementNode.FORWARD_TIMED

                if (MovementNode.direction == MovementNode.NORTH):
                    MovementNode.direction = MovementNode.EAST
                elif (MovementNode.direction == MovementNode.EAST):
                    MovementNode.direction = MovementNode.SOUTH
                elif (MovementNode.direction == MovementNode.SOUTH):
                    MovementNode.direction = MovementNode.WEST
                elif (MovementNode.direction == MovementNode.WEST):
                    MovementNode.direction = MovementNode.NORTH

    def OnBumperCallback(self, data):
        if data.state == 1 and MovementNode.state == MovementNode.MOVING_FORWARD:

            MovementNode.state = MovementNode.MOVING_BACKWARDS
            MovementNode.hit = True

            if (MovementNode.direction == MovementNode.EAST):
                MovementNode.sideways_counter += (MovementNode.last_dist - 0.2)
            elif (MovementNode.direction == MovementNode.WEST):
                MovementNode.sideways_counter -= (MovementNode.last_dist - 0.2)

    def shutdown(self):
        # Stop turtlebot
        rospy.loginfo("Stop TurtleBot")

        # Stop TurtleBot
        self.cmd_vel.publish(Twist())
        # Sleep just makes sure TurtleBot receives the stop command prior to shutting
        # down the script
        rospy.sleep(1)


MovementNode()
