#!/usr/bin/env python3
import sys
import rospy
from std_msgs.msg import String
import moveit_commander
from math import pi


class MoveGroupPythonInteface(object):
    def __init__(self):
        super(MoveGroupPythonInteface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('gripper_control', anonymous=True)

        move_group = moveit_commander.MoveGroupCommander("gripper")
        move_group.set_planning_time(10.0)

        self.move_group = move_group
        rospy.sleep(2)

    def go_to_joint_state(self, j0):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = j0 * pi / 180

        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()


class GripperController:
    def __init__(self):
        rospy.Subscriber("/gripper_command", String, self.callback)
        self.move_group = MoveGroupPythonInteface()
        self.rate = rospy.Rate(10)  # 10hz

    def callback(self, data):
        print("Pose: ", data.data)
        if data.data == 'close':
            self.move_group.go_to_joint_state(76)
        if data.data == 'open':
            self.move_group.go_to_joint_state(0)
        if data.data == 'semi_close':
            self.move_group.go_to_joint_state(52)
        if data.data == 'semi_open':
            self.move_group.go_to_joint_state(39)

    def loop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            if 0xFF == ord('q'):
                break


def main():
    try:
        print("Start")
        gripper = GripperController()
        gripper.loop()

        print("Complete")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
