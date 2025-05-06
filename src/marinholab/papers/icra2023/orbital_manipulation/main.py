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

from math import pi
import numpy as np
import scipy as scipy

from dqrobotics import *
from dqrobotics.interfaces.vrep import DQ_VrepInterface
from dqrobotics.interfaces.vrep.robots import LBR4pVrepRobot
from dqrobotics.solvers import DQ_QuadprogSolver
from dqrobotics.robot_modeling import DQ_Kinematics

from robot_manager import RobotManager
from orbital_manipulation import get_t_om, get_J_t_om, get_J_om

vi = DQ_VrepInterface()

configuration = {
    "eye_radius": 0.25,  # The radius of the eye, defined near Eq. (10)
    "eye_ref": 1,  # The reference frame of the eye, defined in Fig. 3. Usually 1.
    "q_init": [0, pi / 4, 0, pi / 4, 0, pi / 4, 0],
    # The initial joint positions, in this example the same for both robots.
    "T": 0.004,  # The sampling time.
}

try:
    if not vi.connect(19997, 100, 100):
        raise Exception("Error connecting with CoppeliaSim. Be sure that it is open, that port 19997 is open, "
                        "and that the correct scene is opened.")

    vi.start_simulation()
    print("Simulation started.")
    print("Move 'xd1' or 'xd2' in the simulation and see the orbital manipulation algorithm taking care of the rest.")
    print("The rotation of the eye is not being calculated currently, but a physical eye would move as shown in the paper.")
    print("Press CTRL+C to stop.")

    robot_manager_1 = RobotManager()
    robot_manager_2 = RobotManager()

    robot_manager_1.virobot = LBR4pVrepRobot("LBR4p", vi)
    robot_manager_2.virobot = LBR4pVrepRobot("LBR4p#0", vi)

    radius = configuration["eye_radius"]
    first_pair = True
    eye_ref = configuration["eye_ref"]
    q_init = configuration["q_init"]

    # Obtain relevant information from CoppeliaSim to match robot base positions, eye placement, etc
    for robot_manager in [robot_manager_1, robot_manager_2]:
        robot_manager.robot = robot_manager.virobot.kinematics()

        old_eff = robot_manager.robot.get_effector()
        robot_manager.robot.set_effector(old_eff * (1 + 0.5 * E_ * (0.2 * k_)))

        robot_manager.virobot.send_q_target_to_vrep(q_init)
        robot_manager.virobot.send_q_to_vrep(q_init)

        if first_pair:
            x_init = robot_manager.robot.fkm(q_init)
            eye_ref = (1 + 0.5 * E_ * (translation(x_init) - 0.2 * k_ - 0.05 * j_))
            vi.set_object_pose("Sphere", eye_ref)
            first_pair = False

        robot_manager.robot.set_reference_frame(conj(robot_manager.robot.get_reference_frame() * eye_ref))

    first_time = True
    q1 = q_init
    q2 = q_init
    T = configuration["T"]

    robot_manager_1.vrep_xd = "xd1"
    robot_manager_1.vrep_prcm = "prcm1"
    robot_manager_1.q = q1

    robot_manager_2.vrep_xd = "xd2"
    robot_manager_2.vrep_prcm = "prcm2"
    robot_manager_2.q = q2

    for robot_manager in [robot_manager_1, robot_manager_2]:
        vi.set_object_pose(robot_manager.vrep_xd, robot_manager.robot.fkm(robot_manager.q), "Sphere")

    qp_solver = DQ_QuadprogSolver()

    while True:
        x1 = robot_manager_1.robot.fkm(robot_manager_1.q)
        x2 = robot_manager_2.robot.fkm(robot_manager_2.q)

        t1 = translation(x1)
        r1 = rotation(x1)
        line_direction1 = Ad(r1, k_)
        t_om_1 = get_t_om(t1, line_direction1, radius)
        vi.set_object_translation(robot_manager_1.vrep_prcm, t_om_1, "Sphere")

        Jx1 = robot_manager_1.robot.pose_jacobian(robot_manager_1.q)
        Jt1 = robot_manager_1.robot.translation_jacobian(Jx1, x1)
        Jl1 = robot_manager_1.robot.line_jacobian(Jt1, x1, k_)
        J_t_om_1 = get_J_t_om(t1, line_direction1, radius, Jt1, Jl1[0:4, :])
        td1 = vi.get_object_translation(robot_manager_1.vrep_xd, "Sphere")
        e1 = vec4(t1 - td1).reshape(4, 1)

        t2 = translation(x2)
        r2 = rotation(x2)
        line_direction2 = Ad(r2, k_)
        t_om_2 = get_t_om(t2, line_direction2, radius)
        vi.set_object_translation(robot_manager_2.vrep_prcm, t_om_2, "Sphere")

        Jx2 = robot_manager_2.robot.pose_jacobian(robot_manager_2.q)
        Jt2 = robot_manager_2.robot.translation_jacobian(Jx2, x2)
        Jl2 = robot_manager_2.robot.line_jacobian(Jt2, x2, k_)
        J_t_om_2 = get_J_t_om(t2, line_direction2, radius, Jt2, Jl2[0:4, :])
        td2 = vi.get_object_translation(robot_manager_2.vrep_xd, "Sphere")
        e2 = vec4(t2 - td2).reshape(4, 1)

        # Orbital manipulation VFIs
        Jc = scipy.linalg.block_diag(Jt1, Jt2)

        qc = np.vstack((robot_manager_1.q, robot_manager_2.q))
        ec = np.vstack((e1, e2))

        H = Jc.T @ Jc + 0.01 + np.eye(14, 14)
        f = 10 * Jc.T @ ec

        D_om = np.linalg.norm(vec4(t_om_1 - t_om_2)) ** 2
        D_om_minus_safe = 0.025
        D_om_error_minus = D_om - D_om_minus_safe
        W_om_minus = -get_J_om(t_om_1, t_om_2, J_t_om_1, J_t_om_2)
        w_om_minus = np.array([D_om_error_minus])

        D_om_plus_safe = 0.030
        D_om_error_plus = D_om_plus_safe - D_om
        W_om_plus = get_J_om(t_om_1, t_om_2, J_t_om_1, J_t_om_2)
        w_om_plus = np.array([D_om_error_plus])

        # Keep instruments inside eye
        D_safe_eye = (radius - 0.005) ** 2
        D_eye_1 = np.linalg.norm(vec4(t1)) ** 2
        W_eye_1 = np.hstack((DQ_Kinematics.point_to_point_distance_jacobian(Jt1, t1, DQ([0])),
                             np.zeros((1, 7))))
        w_eye_1 = np.array([D_safe_eye - D_eye_1])

        D_eye_2 = np.linalg.norm(vec4(t2)) ** 2
        W_eye_2 = np.hstack((np.zeros((1, 7)),
                             DQ_Kinematics.point_to_point_distance_jacobian(Jt2, t2, DQ([0]))))
        w_eye_2 = np.array([D_safe_eye - D_eye_2])

        W = np.vstack((W_om_minus, W_om_plus, W_eye_1, W_eye_2))
        w = np.vstack((w_om_minus, w_om_plus, w_eye_1, w_eye_2))

        u = qp_solver.solve_quadratic_program(H,
                                              f.reshape(14),
                                              W,
                                              w.reshape(-1),
                                              None,
                                              None)
        u1, u2 = np.split(u, 2)

        robot_manager_1.q = robot_manager_1.q + u1 * T
        robot_manager_2.q = robot_manager_2.q + u2 * T

        robot_manager_1.virobot.send_q_target_to_vrep(robot_manager_1.q)
        robot_manager_1.virobot.send_q_to_vrep(robot_manager_1.q)
        robot_manager_2.virobot.send_q_target_to_vrep(robot_manager_2.q)
        robot_manager_2.virobot.send_q_to_vrep(robot_manager_2.q)

except KeyboardInterrupt:
    print("Interrupted by user")
except Exception as e:
    print("Exception {}.".format(e))
vi.stop_simulation()
vi.disconnect_all()
