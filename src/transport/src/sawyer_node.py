#! /usr/bin/python3
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, MoveGroupFeedback, MoveGroupResult, JointConstraint, OrientationConstraint, WorkspaceParameters
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3, Pose, PoseStamped, Point, Quaternion
from transport.msg import Target
from transport.srv import SawyerCtrl, SawyerCtrlRequest, SawyerCtrlResponse
from path_planner import PathPlanner
import actionlib
import rospy
import intera_interface
import roslib
roslib.load_manifest('transport')


# sawyer_node receives Pick message from <topic> and execute the following sequence:
# (block)
# -> All objects (except the selected one) are added as obstacles
# -> Sawyer moves as planned (from presumably idle position) to an offset above the target object with correct gripper orientation
# -> Sawyer descends to pinch points and grip the target
# -> Sawyer move to a hardcoded distance above center of drop zone and release the target
# Or, if bag is selected:
# -> move to a hardcoded offset to bag handles with upward gripper hook (enforced by gripper orientation)
# -> shift the gripper so that both handles enter the gripper hook
# -> move gripper upward for a hardcoded distance (to secure the bag)
# -> move away from table (prepare for turtle bot transfer)
# Or, if turtle bot is selected, assuming bag is being carried: (can be a stub)
# -> move to hardcoded offset above turtle bot
# -> descend to another hardcoded offset above turtle bot (release bag)
# -> rotate the gripper away from bag handle (hardcoded)
# Or, if selection is out of range, sawyer returns to idle position
# Workspace (table) is hardcoded


class SawyerDirector(object):
    def handler(self, pick: SawyerCtrlRequest) -> SawyerCtrlResponse:
        idle_pose = PoseStamped(
            header=rospy.Header(frame_id="base"),
            pose=Pose(
                position=Point(x=0.69, y=0.16, z=0.38),
                orientation=Quaternion(0, 1, 0, 0)
            )
        )
        try:
            if pick.select >= len(pick.objects):
                # or set joint angles directly?
                return SawyerCtrlResponse(success=True, info="OOR index -> return to idle")
            target: Target = pick.objects[pick.select]
            if target.ident == "block":
                gripper_orient = Quaternion(x=0, y=1, z=0, w=0)
                staging_pose = PoseStamped(
                    header=rospy.Header(frame_id="base"),
                    pose=Pose(
                        # need adjustment for offset on z-axis
                        position=Point(
                            x=target.pose.position.x, y=target.pose.position.y, z=target.pose.position.z + 0.03),
                        orientation=gripper_orient
                    )
                )
                for idx in range(len(pick.objects)):
                    if idx == pick.select:
                        continue
                    obj: Target = pick.objects[idx]
                    self.planner.add_box_obstacle(
                        [obj.dimension.x, obj.dimension.y,
                            obj.dimension.z], f"{obj.ident}-{idx}",
                        PoseStamped(header=rospy.Header(
                            frame_id="base"), pose=obj.pose)
                    )
                # plan = self.planner.plan_to_pose(staging_pose, [], self.SAWYER_FAST, self.SAWYER_FAST)[1]
                # input(f"[DEBUG]")
                # if not self.planner.execute_plan(plan):
                #     raise Exception("Staging execution failed")
                gripping_pose = PoseStamped(
                    header=rospy.Header(frame_id="base"),
                    pose=Pose(
                        position=Point(
                            x=target.pose.position.x, y=target.pose.position.y, z=target.pose.position.z - 0.01),
                        orientation=gripper_orient
                    )
                )
                self.gripper.open()
                plan = self.planner.plan_to_pose(
                    gripping_pose, [], self.SAWYER_FAST, self.SAWYER_SLOW)[1]
                input(f"[DEBUG]")
                if not self.planner.execute_plan(plan):
                    raise Exception("Gripper descend failed")
                self.gripper.close()  # can supply close position/object weight?
                # plan = self.planner.plan_to_pose(staging_pose, [], self.SAWYER_FAST, self.SAWYER_FAST)[1]
                # input(f"[DEBUG]")
                # if not self.planner.execute_plan(plan):
                #     raise Exception("Staging execution failed")
                bag: Target = pick.objects[[
                    obj.ident == "bag" for obj in pick.objects].index(True)]
                gripper_orient = Quaternion(0.5, -0.5, 0.5, 0.5)
                bag_staging_pose = PoseStamped(
                    header=rospy.Header(frame_id="base"),
                    pose=Pose(
                        position=Point(
                            x=bag.pose.position.x, y=bag.pose.position.y, z=bag.pose.position.z + 0.3),
                        orientation=gripper_orient
                    )
                )
                plan = self.planner.plan_to_pose(
                    bag_staging_pose, [], self.SAWYER_FAST, self.SAWYER_SLOW)[1]
                input(f"[DEBUG]")
                if not self.planner.execute_plan(plan):
                    raise Exception("Ready to drop failed")
                # dropping_pose = PoseStamped(
                #     header=rospy.Header(frame_id="base"),
                #     pose=Pose(
                #         position=Point(x=bag.pose.position.x, y=bag.pose.position.y - 0.1, z=bag.pose.position.z - 0.015), # move into the bag
                #         orientation=gripper_orient
                #     )
                # )
                # plan = self.planner.plan_to_pose(dropping_pose, [], self.SAWYER_FAST, self.SAWYER_FAST)[1]
                # input(f"[DEBUG]")
                # if not self.planner.execute_plan(plan):
                #     raise Exception("Move to drop failed")
                self.gripper.open()
                # plan = self.planner.plan_to_pose(bag_staging_pose, [], self.SAWYER_FAST, self.SAWYER_FAST)[1]
                # input(f"[DEBUG]")
                # if not self.planner.execute_plan(plan):
                #     raise Exception("After drop failed")
                self.limb.set_joint_position_speed(0.3)
                joints_cmd = dict(zip(self.limb.joint_names(), [
                                  0, -1, 0, 1, 0, 1.6, 1.57079632679]))
                print(f"Executing {joints_cmd}...")
                self.limb.move_to_joint_positions(joints_cmd, timeout=15)
                self.planner.clear_obstacle()
            elif target.ident == "bag":  # grip the bag
                for idx in range(len(pick.objects)):
                    obj: Target = pick.objects[idx]
                    self.planner.add_box_obstacle(
                        [obj.dimension.x, obj.dimension.y,
                            obj.dimension.z], f"{obj.ident}-{idx}",
                        PoseStamped(header=rospy.Header(
                            frame_id="base"), pose=obj.pose)
                    )
                # go through both handles (hardcoded based on offset to bottom of bag)
                gripper_orient = Quaternion(x=0, y=1, z=0, w=0)
                bag: Target = pick.objects[[
                    obj.ident == "bag" for obj in pick.objects].index(True)]
                hook_pose = PoseStamped(
                    header=rospy.Header(frame_id="base"),
                    pose=Pose(
                        # TODO: x: short edge, y: long edge, offset to avoid contact
                        position=Point(
                            x=bag.pose.position.x, y=bag.pose.position.y - 0.015, z=bag.pose.position.z + 0.175),
                        orientation=gripper_orient
                    )
                )
                while True:
                    plan = self.planner.plan_to_pose(hook_pose, [], self.SAWYER_FAST, self.SAWYER_SLOW)[1]
                    if input(f"y/n? ") == 'y':
                        break
                if not self.planner.execute_plan(plan):
                    raise Exception("Bag staging failed")
                gripper_orient = Quaternion(0, 1, 0, 0)
                gripper_const = [OrientationConstraint(
                    header=rospy.Header(frame_id="base"), link_name="right_gripper_tip", orientation=gripper_orient,
                    absolute_x_axis_tolerance=1.0, absolute_y_axis_tolerance=0.3, absolute_z_axis_tolerance=1.0, weight=1.0
                )]

                self.planner.clear_obstacle()
                hooked_pose = PoseStamped(
                    header=rospy.Header(frame_id="base"),
                    pose=Pose(
                        # TODO: x: short edge, y: long edge, go beyond the length of the bag in x, y directions by grabbing both handles
                        position=Point(
                            x=bag.pose.position.x + 0.125, y=bag.pose.position.y - 0.015, z=bag.pose.position.z + 0.18),
                        orientation=gripper_orient
                    )
                )
                while True:
                    plan = self.planner.plan_to_pose(hooked_pose, [], self.SAWYER_FAST, self.SAWYER_SLOW)[1]
                    if input(f"y/n? ") == 'y':
                        break
                if not self.planner.execute_plan(plan):
                    raise Exception("Bag grip failed")

                while True:
                    plan = self.planner.plan_to_pose(
                        idle_pose, [], self.SAWYER_FAST, self.SAWYER_SLOW)[1]
                    if input(f"y/n? ") == 'y':
                        break
                if not self.planner.execute_plan(plan):
                    raise Exception("Back to idle failed")
                # hooked_pose.pose.position.z += 0.0  # move upward with offset
                # if not self.planner.execute_plan(self.planner.plan_to_pose(hooked_pose, gripper_const, self.SAWYER_FAST, self.SAWYER_FAST)[1]):
                #     raise Exception("Bag secure failed")
                # hooked_pose.pose.position.x
                # hooked_pose.pose.position.y  # set to ranges outside of the table
                # if not self.planner.execute_plan(self.planner.plan_to_pose(hooked_pose, gripper_const, self.SAWYER_FAST, self.SAWYER_FAST)[1]):
                #     raise Exception("Bag secure failed")
            elif target.ident == "bot":  # move to bot and release the bag
                pass
                """ target.pose.position
                gripper_orient = Quaternion(x=, y=, z=, w=)
                gripper_const = [OrientationConstraint(
                    header=rospy.Header(frame_id="base"), link_name="right_gripper", orientation=gripper_orient,
                    absolute_x_axis_tolerance=0.1, absolute_y_axis_tolerance=0.1, absolute_z_axis_tolerance=0.1, weight=1.0
                )]
                drop_pose = PoseStamped(
                    header=rospy.Header(frame_id="base"),
                    pose=Pose(
                        position=Point(x=target.pose.position.x, y=target.pose.position.y, z=target.pose.position.z + ),  # offset above bot with bag height - some const
                        orientation=gripper_orient
                    )
                )
                if not self.planner.execute_plan(self.planner.plan_to_pose(drop_pose, gripper_const, self.SAWYER_FAST, self.SAWYER_FAST)[1]):
                    raise Exception("Bag transport failed")
                drop_pose.pose.position.z -= 0.0  # hardcode descend
                if not self.planner.execute_plan(self.planner.plan_to_pose(drop_pose, gripper_const, self.SAWYER_FAST, self.SAWYER_FAST)[1]):
                    raise Exception("Bag descend failed")
                drop_pose.pose.orientation  # rotate gripper
                if not self.planner.execute_plan(self.planner.plan_to_pose(drop_pose, [], self.SAWYER_FAST, self.SAWYER_FAST)[1]):
                    raise Exception("Bag release failed") """
        except Exception as e:
            return SawyerCtrlResponse(success=False, info=f"[Sawyer] Exception: {str(e)}, Pick: {pick}")
        return SawyerCtrlResponse(success=True, info="")

    def __init__(self):  # params?
        rospy.init_node("sawyer_node")
        self.planner = PathPlanner("right_arm")
        self.gripper = intera_interface.gripper.Gripper(
            "right_gripper", calibrate=True)
        self.gripper.open()
        self.limb = intera_interface.Limb("right")
        try:
            self.SAWYER_FAST = rospy.get_param("~fast_speed")
            self.SAWYER_SLOW = rospy.get_param("~slow_speed")
            # more params?
        except KeyError as e:
            print(f"[Sawyer] Invalid Params: {str(e)}")
            exit()
        rospy.Service("/sawyer_controller", SawyerCtrl, self.handler)
        rospy.spin()

    def shutdown(self):
        self.planner.shutdown()
        pass  # gracefully exit


if __name__ == '__main__':
    SawyerDirector()
