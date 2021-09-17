#!/usr/bin/env python2
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import tf
from pyModbusTCP.client import ModbusClient
import time
# connection Initialization
# c = ModbusClient(host="192.1.1.2", port=502, auto_open=True)


class Gripper:
    '''
    OLD Function
    Availabe commands are:
    gripper_open
    gripper_close
    '''

    def __init__(self, registerNo, _ip="192.1.1.2", _port=502):
        self.regNumber = registerNo
        self.c = ModbusClient(host=_ip, port=_port, auto_open=True)
        # managing TCP sessions with call to c.open()/c.close()
        self.c.open()

    def gripper_open(self):
        self.c.write_single_register(self.regNumber, 0)
        feedback = self.c.read_holding_registers(self.regNumber, 1)
        while feedback[0] is not 0:
            print('Gripper value write error')
            print('Feedback is:', feedback)
            exit

    def gripper_close(self):
        self.c.write_single_register(self.regNumber, 1)
        feedback = self.c.read_holding_registers(self.regNumber, 1)
        while feedback[0] is not 1:
            print('Gripper value write error')
            print('Feedback is:', feedback)
            exit


class moveRobo:
    def __init__(self):
        # init moveit and rospy. Gen node as well
        moveit_commander.roscpp_initialize(sys.argv)

        # get robo's kinetic model and setting up environment for robo
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        # Init group controlling the joints
        self.arm_group = moveit_commander.MoveGroupCommander("arm")

        # to visualize trajectory
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory, queue_size=50
        )

    def getCoordinates(self):
        '''
        Returns the Coordinates and rotation the robot is currently at
        '''
        # start listener
        listener = tf.TransformListener()
        while not rospy.is_shutdown():
            try:
                listener.waitForTransform(
                    "robotiq_arg2f_base_link", "base_link_inertia", rospy.Time(0), rospy.Duration(1))
                (trans, rot) = listener.lookupTransform(
                    "robotiq_arg2f_base_link", "base_link_inertia", rospy.Time(0))
                trans[1] = -trans[1]
                trans[2] = -trans[2]
                rot = [rot[1], rot[0], rot[3], rot[2]]
                return(trans, rot)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print(e)
                continue

    def pos_executor_linear(self, origin_pos, offset_data_euler=([0, 0, 0], [0, 0, 0]), confim=False):
        # TODO might need to change it to subtraction here
        # posEuler = origin_pos[0]+offset_data_euler[0]
        # angleEuler = origin_pos[1]+offset_data_euler[1]
        posEuler = [orig_i + offset_i for orig_i,
                    offset_i in zip(origin_pos[0], offset_data_euler[0])]
        angleEuler = [orig_i + offset_i for orig_i,
                      offset_i in zip(origin_pos[1], offset_data_euler[1])]

        # conversion to radian and then quaterian
        angleEuler = [angleEuler[i]*pi/180 for i in xrange(0, 3)]
        angleQuat = tf.transformations.quaternion_from_euler(
            angleEuler[0], angleEuler[1], angleEuler[2])

        print("Moving to")
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

        self.arm_group.set_pose_target(pose_goal)

        plan = self.arm_group.plan()

        # Housekeeping tasks
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        # Debugging statments
        # eef_link = self.arm_group.get_end_effector_link()
        # print("============ End effector link: %s" % eef_link)

        # print("============ Printing robot state")
        # print(self.arm_group.get_current_state())

        # # Moving Robot to the given point
        if confim:
            self.arm_group.execute(plan, wait=True)

        else:
            inp = raw_input("Continue? y/n: ")[0]
            if (inp == 'y'):
                self.arm_group.execute(plan, wait=True)
            else:
                print "Halting program"


# gripper = Gripper(129)
# gripper.gripper_open()
# time.sleep(5)
# gripper.gripper_close()
if __name__ == '__main__':

    rospy.init_node("move_robo", anonymous=True)
    angleEuler = [0, 90, 0]
    posEuler = [0.3960, 0.0723, 0.3904]
    base = [posEuler, angleEuler]

    robo = moveRobo()
    print(robo.getCoordinates())

    gripper = Gripper(129)
    while 1:
        gripper.gripper_open()
        robo.pos_executor_linear(base, [[0.1, 0.02, 0.1], [10, 0, 0]], True)
        gripper.gripper_close()
        time.sleep(1)
        gripper.gripper_open()
        robo.pos_executor_linear(base, confim=True)
        time.sleep(1)
        gripper.gripper_close()
        time.sleep(1)
