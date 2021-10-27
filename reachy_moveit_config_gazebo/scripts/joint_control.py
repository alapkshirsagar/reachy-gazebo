#!/usr/bin/env python3
import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class Joint:
    def __init__(self, motor_name):
        print('Initializing joint')
        self.name = motor_name
        print('Connecting to', '/' + self.name + '/follow_joint_trajectory')
        self.joint_action = actionlib.SimpleActionClient('/' + self.name + '/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.joint_action.wait_for_server()
        print('Found joint trajectory action!')

    def move_joint(self, angles):
        print('Moving joint')
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = [
            'l_shoulder_pitch',
            'l_shoulder_roll', 
            'l_arm_yaw', 
            'l_elbow_pitch', 
            'l_forearm_yaw', 
            'l_wrist_pitch', 
            'l_wrist_roll'
        ]
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(10)
        goal.trajectory.points.append(point)
        self.joint_action.send_goal_and_wait(goal)

def main():
    arm = Joint('left_arm_position_controller')
    # arm.move_joint([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    arm.move_joint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

if __name__ == '__main__':
    rospy.init_node('joint_position_tester')
    main()



