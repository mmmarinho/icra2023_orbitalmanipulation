"""
All relevant methods implementing the equations for orbital manipulation described in

    Y. Koyama, M. M. Marinho and K. Harada, "Vitreoretinal Surgical Robotic System with Autonomous Orbital
    Manipulation using Vector-Field Inequalities," 2023 IEEE International Conference on Robotics and
    Automation (ICRA), London, United Kingdom, 2023, pp. 4654-4660, doi: 10.1109/ICRA48891.2023.10160795.

Copyright (c) 2023-25 Murilo Marques Marinho (www.murilomarinho.info).
MIT License.
"""

from math import sqrt
import numpy as np
from dqrobotics import *


def get_tip_rcm_distance(t: DQ, line_direction: DQ, radius: float):
    """
    Gets the tip and RCM distance of a given robot.

    Eq. (11) of Y. Koyama, M. M. Marinho and K. Harada, "Vitreoretinal Surgical Robotic System with Autonomous Orbital
    Manipulation using Vector-Field Inequalities," 2023 IEEE International Conference on Robotics and
    Automation (ICRA), London, United Kingdom, 2023, pp. 4654-4660, doi: 10.1109/ICRA48891.2023.10160795.

    :param t: The tooltip position of this robot as a pure quaternion.
    :param line_direction: The shaft line direction as calculated in Eq. (9).
    :param radius: The radius of the eye.
    :return: The tip and RCM distance of a given robot.
    """
    delta = (float(dot(t, line_direction).q[0]) ** 2) - (np.linalg.norm(vec4(t)) ** 2) + (radius ** 2)
    return float(dot(t, line_direction).q[0] + sqrt(delta))


def get_tip_rcm_distance_jacobian(t, l, Jt, Jl):
    """
    Gets the Jacobian relating the current RCM position of a robot being
    controlled using orbital manipulation and its joint velocities. See get_t_om.

    Eq. (18) of Y. Koyama, M. M. Marinho and K. Harada, "Vitreoretinal Surgical Robotic System with Autonomous Orbital
    Manipulation using Vector-Field Inequalities," 2023 IEEE International Conference on Robotics and
    Automation (ICRA), London, United Kingdom, 2023, pp. 4654-4660, doi: 10.1109/ICRA48891.2023.10160795.

    :param t: The tooltip position of this robot as a pure quaternion.
    :param l: The shaft line direction as calculated in Eq. (9).
    :param Jt: The translation Jacobian of the tooltip.
    :param Jl: The line Jacobian of the instrument's shaft.
    :return: The Jacobian relating the current RCM position of a robot being
    controlled using orbital manipulation and its joint velocities
    """
    Jh2 = vec4(l).T @ Jt + vec4(t) @ Jl  # Eq. (16)
    Jh1 = 2 * Jh2 - 2 * vec4(t).T @ Jt  # Eq. (17)
    return np.vstack([Jh2 + Jh1, np.zeros((3, 7))])


def get_t_om(t: DQ, line_direction: DQ, radius: float):
    """
    Get the current RCM position of a robot arm being controlled using
    orbital manipulation.

    Eq. (10) of Y. Koyama, M. M. Marinho and K. Harada, "Vitreoretinal Surgical Robotic System with Autonomous Orbital
    Manipulation using Vector-Field Inequalities," 2023 IEEE International Conference on Robotics and
    Automation (ICRA), London, United Kingdom, 2023, pp. 4654-4660, doi: 10.1109/ICRA48891.2023.10160795.

    :param t: The tooltip position of this robot as a pure quaternion.
    :param line_direction: The shaft line direction as calculated in Eq. (9).
    :param radius: The radius of the eye.
    :return: The current RCM position of a robot arm being controlled using
    orbital manipulation.
    """
    return t - line_direction * get_tip_rcm_distance(t, line_direction, radius)


def get_J_t_om(t: DQ, line_direction: DQ, radius: float, Jt, Jl):
    """
    Get the orbital translation Jacobian of one of the two robotic arms.

    Eq. (19) of Y. Koyama, M. M. Marinho and K. Harada, "Vitreoretinal Surgical Robotic System with Autonomous Orbital
    Manipulation using Vector-Field Inequalities," 2023 IEEE International Conference on Robotics and
    Automation (ICRA), London, United Kingdom, 2023, pp. 4654-4660, doi: 10.1109/ICRA48891.2023.10160795.

    :param t: The tooltip position of this robot as a pure quaternion.
    :param line_direction: The shaft line direction as calculated in Eq. (9).
    :param radius: The radius of the eye.
    :param Jt: The translation Jacobian of the tooltip.
    :param Jl: The line Jacobian of the instrument's shaft.
    :return: The orbital translation Jacobian of this robot.
    """
    tip_rcm_distance = get_tip_rcm_distance(t, line_direction, radius)
    tip_rcm_distance_jacobian = get_tip_rcm_distance_jacobian(t, line_direction, Jt, Jl)
    return Jt - tip_rcm_distance * Jl - hamiplus4(line_direction) @ tip_rcm_distance_jacobian


def get_J_om(t_om_1: DQ, t_om_2: DQ, J_t_om_1, J_t_om_2):
    """
    Get the orbital manipulation Jacobian between two robotic arms.

    Eq. (20) of Y. Koyama, M. M. Marinho and K. Harada, "Vitreoretinal Surgical Robotic System with Autonomous Orbital
    Manipulation using Vector-Field Inequalities," 2023 IEEE International Conference on Robotics and
    Automation (ICRA), London, United Kingdom, 2023, pp. 4654-4660, doi: 10.1109/ICRA48891.2023.10160795.

    :param t_om_1: The result of get_t_om for the first arm.
    :param t_om_2: The result of get_t_om for the second arm.
    :param J_t_om_1: The result of get_J_t_om for the first arm.
    :param J_t_om_2: The result of get_J_t_om for the second arm.
    :return: The orbital manipulation Jacobian between arms.
    """
    return vec4(t_om_1 - t_om_2).T @ np.hstack((J_t_om_1, - J_t_om_2))

def get_D_om(t_om_1: DQ, t_om_2: DQ):
    """
    Get the orbital manipulation squared distance between two robotic arms.

    Eq. (8) of Y. Koyama, M. M. Marinho and K. Harada, "Vitreoretinal Surgical Robotic System with Autonomous Orbital
    Manipulation using Vector-Field Inequalities," 2023 IEEE International Conference on Robotics and
    Automation (ICRA), London, United Kingdom, 2023, pp. 4654-4660, doi: 10.1109/ICRA48891.2023.10160795.

    :param t_om_1: The result of get_t_om for the first arm.
    :param t_om_2: The result of get_t_om for the second arm.
    :return: The orbital manipulation distance between both arms.
    """
    return np.linalg.norm(vec4(t_om_1 - t_om_2)) ** 2