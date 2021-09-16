#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        gripper_ctrl = JointState()
        gripper_ctrl.header = Header()
        gripper_ctrl.header.stamp = rospy.Time.now()
        gripper_ctrl.name = ['finger_joint']
        gripper_ctrl.position = [0.0]
        rospy.loginfo(gripper_ctrl)
        pub.publish(gripper_ctrl)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
