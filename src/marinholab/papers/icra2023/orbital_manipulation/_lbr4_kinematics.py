"""
Given that DQ_VrepInterface was archived, this implements the necessary code from
https://github.com/dqrobotics/cpp-interface-vrep/blob/9d779dc44c0f44aac09bce5cee09d5ceb8b9334d/src/dqrobotics/interfaces/vrep/robots/LBR4pVrepRobot.cpp
"""

from math import pi
import numpy as np

from dqrobotics import *
from dqrobotics.robot_modeling import DQ_SerialManipulatorDH
from dqrobotics.interfaces.coppeliasim import DQ_CoppeliaSimInterfaceZMQ

class LBR4pCoppeliaSimRobot:

    def __init__(self, name: str, ci: DQ_CoppeliaSimInterfaceZMQ):
        self._name = name
        self._joint_names = []
        for i in range(7):
            self._joint_names.append(f"/{name}/joint" + i * "/link/joint")
        self._ci = ci
        self._reference_frame = ci.get_object_pose(self._joint_names[0])

    def kinematics(self):
        """
        See https://github.com/dqrobotics/cpp-interface-vrep/blob/9d779dc44c0f44aac09bce5cee09d5ceb8b9334d/src/dqrobotics/interfaces/vrep/robots/LBR4pVrepRobot.cpp#L73
        """
        pi2 = pi / 2.0

        dh = np.zeros((5, 7))
        dh[1, :] = [0.200, 0, 0.4, 0, 0.39, 0, 0]
        dh[3, :] = [pi2, -pi2, pi2, -pi2, pi2, -pi2, 0]

        kinematics = DQ_SerialManipulatorDH(dh)
        kinematics.set_reference_frame(self._reference_frame)
        kinematics.set_effector(1 + 0.5 * E_ * k_ * 0.07)
        return kinematics

    def send_q_target_to_vrep(self, q):
        self._ci.set_joint_target_positions(self._joint_names, q)

    def send_q_to_vrep(self, q):
        self._ci.set_joint_positions(self._joint_names, q)