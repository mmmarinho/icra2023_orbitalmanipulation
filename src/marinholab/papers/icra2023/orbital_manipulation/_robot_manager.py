"""
Module with RobotManager, that stores information related to each robot for the simulation.

Copyright (c) 2023-25 Murilo Marques Marinho (www.murilomarinho.info).
MIT License.
"""


class RobotManager:
    """
    Holds the relevant data for each robot and avoid repetition.
    """
    def __init(self):
        self.robot = None
        self.virobot = None
        self.vrep_xd = None
        self.vrep_prcm = None
        self.q = None
