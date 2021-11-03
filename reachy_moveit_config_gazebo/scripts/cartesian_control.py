#!/usr/bin/env python3
import rospy
import copy
import actionlib
import sys
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose
from moveit_msgs.msg import MoveGroupAction, DisplayTrajectory
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize

class Arm:
    def __init__(self, controller):
        
        self.group = MoveGroupCommander(controller)
        self.group.allow_replanning(True)
        self.group.set_start_state_to_current_state()
        self.robot = RobotCommander()
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',  DisplayTrajectory, queue_size=20)


    def right_arm_success(self):
        print('Moving joint')
        print(self.group.get_current_pose())
        new_pose = Pose()
        new_pose.position.x = -0.0015629852230885017
        new_pose.position.y = 0.20148619068817522
        new_pose.position.z = 0.3524727872933086
        new_pose.orientation.x = -0.0011823138598739946
        new_pose.orientation.y = 0.0005675124306162963
        new_pose.orientation.z = 0.05229732665315609
        new_pose.orientation.w = 0.998630697349381

        self.group.set_goal_joint_tolerance(0.5)
        self.group.set_goal_orientation_tolerance(0.05)
        self.group.set_goal_position_tolerance(0.5)

        self.group.set_start_state_to_current_state()

        self.group.set_pose_target(new_pose)

        self.group.go(wait=True)
        self.group.stop()

    def trajectory(self):
        print('Moving joint')
        new_pose = Pose()
        new_pose.position.x = -0.0015629852230885017
        new_pose.position.y = 0.20148619068817522
        new_pose.position.z = 0.3524727872933086
        new_pose.orientation.x = -0.0011823138598739946
        new_pose.orientation.y = 0.0005675124306162963
        new_pose.orientation.z = 0.05229732665315609
        new_pose.orientation.w = 0.998630697349381

        self.group.set_goal_joint_tolerance(0.5)
        self.group.set_goal_orientation_tolerance(0.05)
        self.group.set_goal_position_tolerance(0.5)


        self.group.set_start_state_to_current_state()

        end_effector = self.group.get_end_effector_link()

        original_pose = self.group.get_current_pose(end_effector).pose
        waypoints = [original_pose, new_pose, original_pose]

        self.group.compute_cartesian_path(waypoints, 0.01,0.0)

        self.group.go(wait=True)
        self.group.stop()

    def print_current_pose(self):
        print(self.group.get_current_pose())

    def left_arm_success(self):
        print('Moving joint')
        new_pose = Pose()
        new_pose.orientation.x = 0.29561313709800985
        new_pose.orientation.y = -0.00118967076273227
        new_pose.orientation.z = 0.7051219976533809
        new_pose.orientation.x = -0.4928211273047975
        new_pose.orientation.y = -0.2919778309320321
        new_pose.orientation.z = 0.30009606689292934
        new_pose.orientation.x= 0.7627703673856562

        self.group.set_goal_joint_tolerance(0.5)
        self.group.set_goal_orientation_tolerance(0.05)
        self.group.set_goal_position_tolerance(0.5)

        self.group.set_start_state_to_current_state()

        self.group.set_pose_target(new_pose)

        self.group.go(wait=True)
        self.group.stop()


def main():

    left_arm = Arm("left_arm")
    left_arm.left_arm_success()

    right_arm = Arm("right_arm")
    right_arm.right_arm_success()

if __name__ == '__main__':
    
    rospy.init_node('arm_tester')
    main()



