#!/usr/bin/env python
import rospy
from intera_interface import Limb, gripper
from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

import numpy as np

import ros_numpy
from planning.src.path_planner import PathPlanner
from planning.src.controller import Controller
from sensing.srv import ObjectSrv


def confirm():
    ans = input("Confirm next move (y/n)")
    if ans == 'n':
        raise Exception("Canceled plan")


class MoveItAgent:
    def __init__(self, use_pid=False):
        rospy.init_node('moveit_agent')

        if not self._load_parameters():
            return

        self.planner = PathPlanner("right_arm")

        self.use_pid = use_pid
        Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
        Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
        Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
        Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
        self.controller = Controller(Kp, Ki, Kd, Kw, Limb("right"))

        self.gripper = gripper.Gripper('right_gripper')
        self._calibrate_gripper()
    
    def run(self):
        # dependency services
        rospy.wait_for_service(self._object_service)

        object_proxy = rospy.ServiceProxy(self._object_service, ObjectSrv)
        objects = object_proxy().objects

        for object in objects:
            self._move_arm(ros_numpy.numpify(object))
            self._close_gripper()
            self._reset()
            # TODO: to bag
            self._open_gripper()
            self._reset()

    def _move_arm(self, target_position, target_orientation=[1.0, 0.0, 0.0, 0.0], orien_const=None):
        while not rospy.is_shutdown():
            try:
                x, y, z = target_position
                goal = PoseStamped()
                goal.header.frame_id = self._base_frame

                #x, y, and z position
                goal.pose.position.x = x
                goal.pose.position.y = y
                goal.pose.position.z = z

                #Orientation as a quaternion
                x, y, z, w = target_orientation
                goal.pose.orientation.x = x
                goal.pose.orientation.y = y
                goal.pose.orientation.z = z
                goal.pose.orientation.w = w

                if orien_const:
                    plan = self.planner.plan_to_pose(goal, [orien_const], [])
                else:
                    plan = self.planner.plan_to_pose(goal, [], [])
                
                confirm()

                if self.use_pid:
                    if not self.controller.execute_path(plan[1]):
                        raise Exception("Execution failed")
                else:
                    if not self.planner.execute_path(plan[1]):
                        raise Exception("Execution failed")
            except Exception as e:
                print(e)
            else:
                break

    def _reset(self):
        # neutral_state = np.array([...])
        # forward kinematics to this position
        pass

    def _calibrate_gripper(self):
        print('Calibrating...')
        self.gripper.calibrate()
        rospy.sleep(1.0)

    def _open_gripper(self):
        confirm()
        print('Opening...')
        self.gripper.open()
        rospy.sleep(1.0)

    def _close_gripper(self):
        confirm()
        print('Closing...')
        self.gripper.close()
        rospy.sleep(1.0)

    def _load_parameters(self):
        try:
            self._object_service = rospy.get_param('services/object')

            self._base_frame = rospy.get_param('frames/base')
            self._head_frame = rospy.get_param('frames/head')
            self._table_frame = rospy.get_param('frames/table')
            return True
        except rospy.ROSException as e:
            print(e)
            return False


if __name__ == "__main__":
    node = MoveItAgent()
    if node:
        node.run()
