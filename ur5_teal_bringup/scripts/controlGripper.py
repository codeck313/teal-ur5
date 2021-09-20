#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String


class moveGripper:
    def __init__(self):
        self.pub = rospy.Publisher(
            'finger_joint_controller', String, queue_size=10)
        # rospy.sleep(1)
        while self.pub.get_num_connections() < 1:
            pass

    def update_gripper(self, status):
        hello_str = status
        rospy.loginfo(hello_str)
        # for i in xrange(0, 100):
        self.pub.publish(hello_str)
        # rate = rospy.Rate(10)
        # while not rospy.is_shutdown():
        #     hello_str = status
        #     rospy.loginfo(hello_str)
        #     pub.publish(hello_str)
        #     rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('test_finger_joint_controllers', anonymous=True)
        gripper = moveGripper()
        gripper.update_gripper('close')
    except rospy.ROSInterruptException:
        pass
