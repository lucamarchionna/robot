#!/usr/bin/env python

import sys

import rospy
import moveit_commander

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose

from geometry_representation import GeometryParser, GeometryPose

from edocontroller.edo_abstract_class import EdoAbstractClass


class DangerousPositionError(Exception):
    pass


class MoveitController(EdoAbstractClass):
    """This class is an upper layer to moveit to ease the rest of the development,
    It has the responsibility to calculate trajectories and ask for the movement to be done
    Moreover, all poses given to this class should be in the fiducial coordinate system
    This class uses mapper_env_moveit to transform the fiducial coordinate system to the moveit one"""

    def __init__(self):

        # moveit attributes
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "edo"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        self.channel_gripper = rospy.Publisher('/open_gripper', Bool, queue_size=3)

        self.move_group.set_end_effector_link("edo_link_ee")

    def get_current_joint_values(self):
        return self.move_group.get_current_joint_values()

    def get_current_pose(self):
        """WARNING : This gives the pose seen by Moveit
        TODO : Especially, the end effector is not the gripper in this pose, and the coordinate are in the moveit
        TODO : coordinate system"""
        current_pose = self.move_group.get_current_pose().pose
        return GeometryParser.geometry_pose_from_pose(current_pose)

    def go_to_joint(self, joint1, joint2, joint3, joint4, joint5, joint6):
        """joints in radiant"""
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = joint1
        joint_goal[1] = joint2
        joint_goal[2] = joint3
        joint_goal[3] = joint4
        joint_goal[4] = joint5
        joint_goal[5] = joint6

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

    def _correct_pose_goal(self, pose_goal_geometry):
        """The length of the gripper is not taken into account in moveit, so we need to correct the value
        To correct the value, we measure the length of the gripper in real world
        TODO find a way to do the correction without using the real world and hardcoding gripper length value"""

        return pose_goal_geometry

    def go_to_pose_goal(self, pose_goal):
        """pose_goal_geometry type should be a GeometryPose object or a Pose object, where the coordinate and rotation are given
        in the fiducial coordinate system"""

        pose_goal_geometry = pose_goal
        if isinstance(pose_goal, Pose):
            pose_goal_geometry = GeometryParser.geometry_pose_from_pose(pose_goal)
        else:
            if not isinstance(pose_goal, GeometryPose):
                raise ValueError("argument should be of type GeometryPose or Pose")

        pose_goal_geometry.orientation = pose_goal_geometry.orientation.normalized()
        pose_goal_geometry = self._correct_pose_goal(pose_goal_geometry)
        pose_goal_message = GeometryParser.pose_from_geometry_pose(pose_goal_geometry)

        self._go_to_moveit_pose_goal(pose_goal_message)

    def _prevent_collision_with_ground(self, pose_goal_in_moveit):
        # TODO : This function depends on the environment and of the END_EFFECTOR.
        # TODO : It should be checked and modified so that it is safe in your environment
        pass

    def _go_to_moveit_pose_goal(self, pose_goal_in_moveit):
        """This function should always be called to go to a pose_goal.
        It send an order to move_it, after ensuring it is safe for the robot to go there

        the pose goal is a Pose message
        """

        self._prevent_collision_with_ground(pose_goal_in_moveit)

        self.move_group.set_goal_tolerance(0.005)
        self.move_group.set_pose_target(pose_goal_in_moveit)

        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def reset_position(self):
        """put all the joints values to 0"""
        self.go_to_joint(0, 0, 0, 0, 0, 0)

    def close_gripper(self):
        rate = rospy.Rate(5)
        for i in range(3):
            self.channel_gripper.publish(Bool(False))
            rate.sleep()

    def open_gripper(self):
        rate = rospy.Rate(10)
        for i in range(4):
            self.channel_gripper.publish(Bool(True))
            rate.sleep()


    def set_velocity(self,ovr):
        self.move_group.set_max_velocity_scaling_factor(ovr)
