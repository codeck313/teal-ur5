#! /usr/bin/env python

import time
import rospy

# Brings in the SimpleActionClient
import actionlib
from ur5_gripper import moveRobo
from controlGripper import moveGripper
# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from teal_camera.msg import ur5_cameraAction, ur5_cameraGoal


def camera_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('camera_server', ur5_cameraAction)
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = ur5_cameraGoal(start=1)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()
    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult


if __name__ == '__main__':
    homeAngleEuler = [0, 90, 0]
    homePosEuler = [0.3960, 0.0723, 0.3904]
    home = [homePosEuler, homeAngleEuler]

    cameraAngleEuler = [0, 90, 0]
    cameraPosEuler = [0.3960, 0.0723, 0.3904]
    camera = [cameraPosEuler, cameraAngleEuler]

    workAngleEuler = [-180, 0, -180]
    workPosEuler = [0.3203, 0.1160, 0.1775]
    work1 = [workPosEuler, workAngleEuler]

    workPosEuler = [0.3203, 0.1160, 0.2500]
    work2 = [workPosEuler, workAngleEuler]

    dropAngleEuler = [-180, 0, -180]
    dropPosEuler1 = [0.3216, -0.2238, 0.2500]
    drop1 = [dropPosEuler1, dropAngleEuler]

    dropPosEuler2 = [0.3216, -0.2238, 0.1775]
    drop2 = [dropPosEuler2, dropAngleEuler]

    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('robot_server')
        result = camera_client()
        print "Result from camera:", str(result.x/100.0), str(result.y /
                                                              100.0), str(result.angle)
        robo = moveRobo()
        print("Current pos")
        print(robo.getCoordinates())
        time.sleep(1)
        gripper = moveGripper()

        # Home
        rospy.loginfo(
            "**********************Moving to Home Position**********************")
        robo.pos_executor_linear(home)
        gripper.update_gripper("open")

        # Camera Position
        rospy.loginfo(
            "**********************Moving to Camera Position**********************")
        robo.pos_executor_linear(camera)
        rospy.loginfo(
            "**********************Asking cameraRobo for coordinates**********************")

        # Grab Object
        rospy.loginfo(
            "**********************Grabbing the object**********************")
        robo.pos_executor_linear(
            work1, [[result.x/100.0, -result.y/100.0, 0], [0, 0, result.angle]])
        gripper.update_gripper("close")
        robo.pos_executor_linear(
            work2, [[result.x/100.0, -result.y/100.0, 0], [0, 0, result.angle]])
        gripper.update_gripper("close")

        # Drop Object
        rospy.loginfo(
            "**********************Dropping the object**********************")
        robo.pos_executor_linear(drop1)
        robo.pos_executor_linear(drop2)

        # for safety
        time.sleep(2)
        gripper.update_gripper("open")

    except rospy.ROSInterruptException:
        print "Program interrupted before completion"
