"""
MIT License

Copyright (c) 2023 Murilo Marques Marinho (www.murilomarinho.info)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

from math import sqrt
import numpy as np
from dqrobotics import *


def get_tip_rcm_distance(t: DQ, line_direction: DQ, radius: float):
    """
    Gets the tip and RCM distance of a given robot.
    Eq. (11) of Vitreoretinal Surgical Robotic System with Autonomous Orbital
    Manipulation using Vector-Field Inequalities. Koyama, Y.; Marinho, M. M.;
    and Harada, K. In 2023 IEEE International Conference on Robotics
    and Automation (ICRA), pages 1–7, May 2023
    :param t: The tooltip positon of this robot as a pure quaternion.
    :param line_direction: the shaft line direction as calculated in Eq. (9).
    :param radius: the radius of the eye.
    :return: the tip and RCM distance of a given robot.
    """
    delta = (float(dot(t, line_direction).q[0]) ** 2) - (np.linalg.norm(vec4(t)) ** 2) + (radius ** 2)
    return float(dot(t, line_direction).q[0] + sqrt(delta))


def get_tip_rcm_distance_jacobian(t, l, Jt, Jl):
    """
    Gets the Jacobian relating the current RCM position of a robot being
    controlled using orbital manipulation and its joint velocities.
    See get_t_om.
    Eq. (18) of Vitreoretinal Surgical Robotic System with Autonomous Orbital
    Manipulation using Vector-Field Inequalities. Koyama, Y.; Marinho, M. M.;
    and Harada, K. In 2023 IEEE International Conference on Robotics
    and Automation (ICRA), pages 1–7, May 2023
    :param t: the tooltip positon of this robot as a pure quaternion.
    :param l: the shaft line direction as calculated in Eq. (9).
    :param Jt: the translation Jacobian of the tooltip.
    :param Jl: the line Jacobian of the instrument's shaft.
    :return: the Jacobian relating the current RCM position of a robot being
    controlled using orbital manipulation and its joint velocities
    """
    Jh2 = vec4(l).T @ Jt + vec4(t) @ Jl  # Eq. (16)
    Jh1 = 2 * Jh2 - 2 * vec4(t).T @ Jt  # Eq. (17)
    return np.vstack([Jh2 + Jh1, np.zeros((3, 7))])


def get_t_om(t: DQ, line_direction: DQ, radius: float):
    """
    Obtain the current RCM position of a robot arm being controlled using
    orbital manipulation.
    Eq. (10) of Vitreoretinal Surgical Robotic System with Autonomous Orbital
    Manipulation using Vector-Field Inequalities. Koyama, Y.; Marinho, M. M.;
    and Harada, K. In 2023 IEEE International Conference on Robotics
    and Automation (ICRA), pages 1–7, May 2023
    :param t: the tooltip positon of this robot as a pure quaternion.
    :param line_direction: the shaft line direction as calculated in Eq. (9).
    :param radius: the radius of the eye.
    :return: the current RCM position of a robot arm being controlled using
    orbital manipulation.
    """
    return t - line_direction * get_tip_rcm_distance(t, line_direction, radius)


def get_J_t_om(t: DQ, line_direction: DQ, radius: float, Jt, Jl):
    """
    Obtain the orbital translation Jacobian of one of the two robotic arms.
    Eq. (19) of Vitreoretinal Surgical Robotic System with Autonomous Orbital
    Manipulation using Vector-Field Inequalities. Koyama, Y.; Marinho, M. M.;
    and Harada, K. In 2023 IEEE International Conference on Robotics
    and Automation (ICRA), pages 1–7, May 2023
    :param t: The tooltip positon of this robot as a pure quaternion.
    :param line_direction: the shaft line direction as calculated in Eq. (9).
    :param radius: the radius of the eye.
    :param Jt: the translation Jacobian of the tooltip.
    :param Jl: the line Jacobian of the instrument's shaft.
    :return: the orbital translation Jacobian of this robot.
    """
    tip_rcm_distance = get_tip_rcm_distance(t, line_direction, radius)
    tip_rcm_distance_jacobian = get_tip_rcm_distance_jacobian(t, line_direction, Jt, Jl)
    return Jt - tip_rcm_distance * Jl - hamiplus4(line_direction) @ tip_rcm_distance_jacobian


def get_J_om(t_om_1: DQ, t_om_2: DQ, J_t_om_1, J_t_om_2):
    """
    Obtain the orbital manipulation Jacobian between two robotic arms.
    Eq. (20) of Vitreoretinal Surgical Robotic System with Autonomous Orbital
    Manipulation using Vector-Field Inequalities. Koyama, Y.; Marinho, M. M.;
    and Harada, K. In 2023 IEEE International Conference on Robotics
    and Automation (ICRA), pages 1–7, May 2023
    :param t_om_1: the result of get_t_om for the first arm.
    :param t_om_2: the result of get_t_om for the second arm.
    :param J_t_om_1: the result of get_J_t_om for the first arm.
    :param J_t_om_2: the result of get_J_t_om for the second arm.
    :return: The orbital manipulation Jacobian between arms.
    """
    return vec4(t_om_1 - t_om_2).T @ np.hstack((J_t_om_1, - J_t_om_2))

def get_D_om(t_om_1: DQ, t_om_2: DQ):
    """
    Obtain the orbital manipulation squared distance between two robotic arms.
    Eq. (8) of Vitreoretinal Surgical Robotic System with Autonomous Orbital
    Manipulation using Vector-Field Inequalities. Koyama, Y.; Marinho, M. M.;
    and Harada, K. In 2023 IEEE International Conference on Robotics
    and Automation (ICRA), pages 1–7, May 2023
    :param t_om_1: the result of get_t_om for the first arm.
    :param t_om_2: the result of get_t_om for the second arm.
    :return: the orbital manipulation distance between both arms.
    """
    return np.linalg.norm(vec4(t_om_1 - t_om_2)) ** 2