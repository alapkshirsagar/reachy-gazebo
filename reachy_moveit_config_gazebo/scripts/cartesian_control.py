#!/usr/bin/env python3
import rospy
import actionlib
import sys
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose
from moveit_msgs.msg import MoveGroupAction, DisplayTrajectory
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize

class Arm:
    def __init__(self):
        
        self.group = MoveGroupCommander("right_arm")
        self.group.allow_replanning(True)
        self.group.set_start_state_to_current_state()
        self.robot = RobotCommander()
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',  DisplayTrajectory, queue_size=20)
        #rospy.sleep(10)
    
    # def __init__(self):
    #     print('Initializing joint')
    #     print('Connecting to', '/move_group')
    #     self.move_client = actionlib.SimpleActionClient('/move_group', MoveGroupAction)
    #     self.move_client.wait_for_server()

    #     roscpp_initialize(sys.argv)

    #     group_name = RobotCommander().get_group_names()[0]
    #     group = MoveGroupCommander("right arm")


        
    #     print('Found move client!')

    def move_arm(self):
        print('Moving joint')
        pose_target = Pose()
        pose_target.orientation.x = -0.002769137926803048
        pose_target.orientation.y = 0.0024013303286062925
        pose_target.orientation.z = -0.039698277529881615
        pose_target.orientation.w = 0.9992049901041106
        pose_target.position.x = -0.001338614902623077
        pose_target.position.y = 0.19912895362063396
        pose_target.position.z = 0.3524417531161036
        end_effector = self.group.get_end_effector_link()
        pose = self.group.get_current_pose(end_effector)
        print("pose before")
        print(pose)
        pose.pose.position.z += 0.2
        pose.pose.position.x += 0.2

        print("pose after")
        print(pose)
        self.group.set_pose_target(pose, end_effector)
        print(self.group.get_current_pose(end_effector))

        self.group.set_goal_tolerance(0.1)

        self.group.go(wait=True)

        self.group.stop()

        # plan = self.group.plan()

        # display_trajectory = DisplayTrajectory()
        # display_trajectory.trajectorystart = self.robot.get_current_state()
        # display_trajectory.trajectory.append(plan)
        # self.display_trajectory_publisher.publish(display_trajectory)

def main():
    arm = Arm()
    arm.move_arm()

if __name__ == '__main__':
    
    rospy.init_node('arm_tester')
    main()



