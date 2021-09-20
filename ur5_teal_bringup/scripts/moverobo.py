#!/usr/bin/env python2
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import tf


# to change pos
# template for zero pos
# angleEuler = [0, 180, 0]
# posEuler = [0.5, 0.000, 0.1790]
# posEuler = [0.665, 0, 0.1790]


# template for camera capture
# angleEuler = [0, 90, 0]
# posEuler = [0.396, -0.073, 0.735]

# NEW template for camera capture
angleEuler = [0, 90, 0]
posEuler = [0.3960, 0.0723, 0.3940]

# init moveit and rospy. Gen node as well
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_robo", anonymous=True)

# get robo's kinetic model and setting up environment for robo
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

# Init group controlling the joints
arm_group = moveit_commander.MoveGroupCommander("arm")
# gripper_group = moveit_commander.MoveGroupCommander("gripper")

# to visualize trajectory
display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory, queue_size=50
)


# start listener
listener = tf.TransformListener()
while not rospy.is_shutdown():
    try:
        listener.waitForTransform(
            "robotiq_arg2f_base_link", "base_link_inertia", rospy.Time(0), rospy.Duration(1))
        (trans, rot) = listener.lookupTransform(
            "robotiq_arg2f_base_link", "base_link_inertia", rospy.Time(0))
        break
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print(e)
        continue


angleEuler = [angleEuler[i]*pi/180 for i in xrange(0, 3)]
angleQuat = tf.transformations.quaternion_from_euler(
    angleEuler[0], angleEuler[1], angleEuler[2])

# copy the position
# posEuler = trans

# copy the orientation
# angleQuat = rot
# angleEuler = tf.transformations.euler_from_quaternion(angleQuat)
# angleEuler = [angleEuler[i]*180/pi for i in xrange(0, 3)]

print(posEuler)
print(angleEuler)
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.x = -angleQuat[0]
pose_goal.orientation.y = angleQuat[1]
pose_goal.orientation.z = angleQuat[2]
pose_goal.orientation.w = angleQuat[3]

pose_goal.position.x = posEuler[0]
pose_goal.position.y = posEuler[1]
pose_goal.position.z = posEuler[2]

arm_group.set_pose_target(pose_goal)
# plan = move_group.go(wait=True)

plan = arm_group.plan()
# rospy.sleep(5)

# Calling `stop()` ensures that there is no residual movement
arm_group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
arm_group.clear_pose_targets()
# joint_goal = gripper_group.get_current_joint_values()
# print("Joint")
# print(joint_goal)
# joint_goal[0] = 0.8
# joint_goal[1] = 0.8
# gripper_group.go(joint_goal, wait=True)
# gripper_group.stop()
inp = raw_input("Continue? y/n: ")[0]
if (inp == 'y'):
    arm_group.execute(plan, wait=True)
else:
    print "Halting program"
