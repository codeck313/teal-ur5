#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import String
from pyModbusTCP.client import ModbusClient


class Gripper:

    def __init__(self, registerNo, _ip="192.1.1.2", _port=502):
        self.regNumber = registerNo
        self.c = ModbusClient(host=_ip, port=_port, auto_open=True)
        # managing TCP sessions with call to c.open()/c.close()
        self.c.open()
        self.grip = 0
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        # rospy.sleep(1)
        while self.pub.get_num_connections() < 1:
            pass

    def gripper_open(self):
        self.c.write_single_register(self.regNumber, 0)
        feedback = self.c.read_holding_registers(self.regNumber, 1)
        while feedback[0] is not 0:
            print('Gripper value write error')
            print('Feedback is:', feedback)
            self.c.write_single_register(self.regNumber, 0)
            feedback = self.c.read_holding_registers(self.regNumber, 1)

    def gripper_close(self):
        self.c.write_single_register(self.regNumber, 1)
        feedback = self.c.read_holding_registers(self.regNumber, 1)
        while feedback[0] is not 1:
            print('Gripper value write error')
            print('Feedback is:', feedback)
            self.c.write_single_register(self.regNumber, 1)
            feedback = self.c.read_holding_registers(self.regNumber, 1)

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        if data.data == "close":
            self.grip = 1
            self.gripper_close()
            self.updateRviz()
        else:
            self.grip = 0
            self.gripper_open()
            self.updateRviz()

    def start(self):
        rospy.Subscriber("finger_joint_controller", String, self.callback)
        for i in xrange(0, 10):
            self.updateRviz()
        # rospy.spin()
        rate = rospy.Rate(10)  # Run at 10Hz
        while not rospy.is_shutdown():
            rate.sleep()

    def updateRviz(self):
        gripper_ctrl = JointState()
        gripper_ctrl.header = Header()
        gripper_ctrl.header.stamp = rospy.Time.now()
        gripper_ctrl.name = ['finger_joint']
        gripper_ctrl.position = [self.grip*0.8]
        rospy.loginfo(gripper_ctrl)
        self.pub.publish(gripper_ctrl)


if __name__ == '__main__':
    rospy.init_node('finger_joint_controller', anonymous=True)
    try:
        grip = Gripper(129)
        grip.start()
        # grip.updateRviz()

    except rospy.ROSInterruptException:
        pass
