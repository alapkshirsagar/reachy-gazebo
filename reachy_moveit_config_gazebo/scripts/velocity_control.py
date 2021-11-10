#!/usr/bin/env python3

# from control_msgs.msg import *
# from reachy import Reachy, parts


# reachy = Reachy(
#     left_arm=parts.LeftArm(io='/dev/ttyUSB*', hand='force_gripper'),
# )


#     # # Speed
#     # @property
#     # def moving_speed(self):
#     #     """Get the maximum speed (in degree per second) of the motor."""
#     #     return self._motor.target_rot_speed

#     # @moving_speed.setter
#     # def moving_speed(self, value):
#     #     self._motor.target_rot_speed = value



# for m in reachy.left_arm.motors:
#     m.moving_speed()

from reachy_sdk import ReachySDK

reachy=ReachySDK('127.0.0.1')


for name, joint in reachy.joints.items():
    print(joint.present_position)

