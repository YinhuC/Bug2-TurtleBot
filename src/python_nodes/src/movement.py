#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent  # Import the bumper message

PI = 3.1415926535897


class MovementNode():

    # Define some state constants so we know what is happening
    MOVING_FORWARD = 0
    MOVING_BACKWARDS = 1
    TURNING = 2

    # Current state of the robot
    state = MOVING_FORWARD
    corner_count = 0

    sideways_counter = 0

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

        perpendicular = 90*2*PI/360
        angle_relative = 0
        distance_relative = 0

        # As long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():

            timeStart = rospy.Time.now().to_sec()
            while MovementNode.state == MovementNode.MOVING_FORWARD:
                move_cmd.linear.x = 0.2
                move_cmd.angular.z = 0.0

                self.cmd_vel.publish(move_cmd)
                r.sleep()

                timeNow = rospy.Time.now().to_sec()
                distance_relative = 0.2*(timeNow-timeStart)

                if ((MovementNode.corner_count > 0) and (distance_relative > 1) and (MovementNode.corner_count != 3)) or \
                        ((MovementNode.corner_count == 3) and (distance_relative > 2)):

                    if(MovementNode.corner_count == 2):
                        MovementNode.sideways_counter += distance_relative
                        MovementNode.state = MovementNode.TURNING

                    elif(MovementNode.corner_count == 4):
                        MovementNode.sideways_counter -= distance_relative
                        MovementNode.state = MovementNode.TURNING

                    elif(MovementNode.corner_count == 3):
                        MovementNode.state = MovementNode.TURNING

            timeStart = rospy.Time.now().to_sec()
            while MovementNode.state == MovementNode.MOVING_BACKWARDS:
                move_cmd.linear.x = -0.2
                move_cmd.angular.z = 0.0

                self.cmd_vel.publish(move_cmd)
                r.sleep()

                timeNow = rospy.Time.now().to_sec()
                distance_relative = 0.2*(timeNow-timeStart)

                if(distance_relative > 0.5):
                    MovementNode.state = MovementNode.TURNING

            timeStart = rospy.Time.now().to_sec()
            while MovementNode.state == MovementNode.TURNING:

                if (MovementNode.corner_count == 1):
                    move_cmd.linear.x = 0.0
                    move_cmd.angular.z = 1.8

                elif (MovementNode.corner_count == 2) or (MovementNode.corner_count == 3):
                    move_cmd.linear.x = 0.0
                    move_cmd.angular.z = -1.8

                elif (MovementNode.corner_count == 4):
                    move_cmd.linear.x = 0.0
                    move_cmd.angular.z = 1.8

                self.cmd_vel.publish(move_cmd)
                r.sleep()

                timeNow = rospy.Time.now().to_sec()
                angle_relative = (1.8)*(timeNow-timeStart)

                if(angle_relative > perpendicular):
                    MovementNode.state = MovementNode.MOVING_FORWARD

                    if (MovementNode.corner_count == 1) or (MovementNode.corner_count == 2) or (MovementNode.corner_count == 3):
                        MovementNode.corner_count += 1
                    elif (MovementNode.corner_count == 4):
                        MovementNode.corner_count = 0

    def OnBumperCallback(self, data):
        if data.state == 1 and MovementNode.state == MovementNode.MOVING_FORWARD:
            rospy.loginfo('Ouch - I hit something!')

            if (MovementNode.corner_count == 0):
                MovementNode.corner_count += 1
                MovementNode.state = MovementNode.MOVING_BACKWARDS

            elif (MovementNode.corner_count == 2) and (MovementNode.corner_count == 3):
                MovementNode.corner_count -= 1
                MovementNode.state = MovementNode.MOVING_FORWARD

    def shutdown(self):
        # Stop turtlebot
        rospy.loginfo("Stop TurtleBot")

        # Stop TurtleBot
        self.cmd_vel.publish(Twist())
        # Sleep just makes sure TurtleBot receives the stop command prior to shutting
        # down the script
        rospy.sleep(1)


MovementNode()
