#!/usr/bin/env python3

# Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
# Convenience code to wrap TRAC IK

from trac_ik_python.trac_ik_wrap import TRAC_IKSolver

try:
    import rclpy
except ImportError:  # pragma: no cover
    rclpy = None


class IK(object):
    def __init__(self, base_link, tip_link,
                 timeout=0.005, epsilon=1e-5, solve_type="Speed",
                 urdf_string=None, node=None,
                 robot_description_parameter="robot_description"):
        """
        Create a TRAC_IK instance and keep track of it.

        :param str base_link: Starting link of the chain.
        :param str tip_link: Last link of the chain.
        :param float timeout: Timeout in seconds for the IK calls.
        :param float epsilon: Error epsilon.
        :param solve_type str: Type of solver, can be:
            Speed (default), Distance, Manipulation1, Manipulation2
        :param urdf_string str: Optional URDF XML string.
        :param node: Optional ROS 2 rclpy node used to fetch robot description.
        :param str robot_description_parameter: ROS 2 parameter name used when
            urdf_string is not provided.
        """
        if urdf_string is None:
            urdf_string = self._resolve_urdf_string(node, robot_description_parameter)
        self._urdf_string = urdf_string
        self._timeout = timeout
        self._epsilon = epsilon
        self._solve_type = solve_type
        self.base_link = base_link
        self.tip_link = tip_link
        self._ik_solver = TRAC_IKSolver(self.base_link,
                        self.tip_link,
                        self._urdf_string,
                        self._timeout,
                        self._epsilon,
                        self._solve_type)
        self.number_of_joints = self._ik_solver.getNrOfJointsInChain()
        self.joint_names = self._ik_solver.getJointNamesInChain(
            self._urdf_string)
        self.link_names = self._ik_solver.getLinkNamesInChain()

    @staticmethod
    def _resolve_urdf_string(node, robot_description_parameter):
        if node is None:
            raise ValueError(
                "urdf_string is required in ROS 2 unless a node is provided "
                "to read the robot description parameter"
            )

        if rclpy is None:
            raise ImportError("rclpy is required to fetch robot_description in ROS 2")

        if not node.has_parameter(robot_description_parameter):
            node.declare_parameter(robot_description_parameter, "")

        urdf_string = node.get_parameter(robot_description_parameter).value
        if not urdf_string:
            raise ValueError(
                "robot description parameter '%s' is empty" % robot_description_parameter
            )

        return urdf_string

    def get_ik(self, qinit,
               x, y, z,
               rx, ry, rz, rw,
               bx=1e-5, by=1e-5, bz=1e-5,
               brx=1e-3, bry=1e-3, brz=1e-3):
        """
        Do the IK call.

        :param list of float qinit: Initial status of the joints as seed.
        :param float x: X coordinates in base_frame.
        :param float y: Y coordinates in base_frame.
        :param float z: Z coordinates in base_frame.
        :param float rx: X quaternion coordinate.
        :param float ry: Y quaternion coordinate.
        :param float rz: Z quaternion coordinate.
        :param float rw: W quaternion coordinate.
        :param float bx: X allowed bound.
        :param float by: Y allowed bound.
        :param float bz: Z allowed bound.
        :param float brx: rotation over X allowed bound.
        :param float bry: rotation over Y allowed bound.
        :param float brz: rotation over Z allowed bound.

        :return: joint values or None if no solution found.
        :rtype: tuple of float.
        """
        if len(qinit) != self.number_of_joints:
            raise Exception("qinit has length %i and it should have length %i" % (
                len(qinit), self.number_of_joints))
        solution = self._ik_solver.CartToJnt(qinit,
                                             x, y, z,
                                             rx, ry, rz, rw,
                                             bx, by, bz,
                                             brx, bry, brz)
        if solution:
            return solution
        else:
            return None

    def get_joint_limits(self):
        """
        Return lower bound limits and upper bound limits for all the joints
        in the order of the joint names.
        """
        lb = self._ik_solver.getLowerBoundLimits()
        ub = self._ik_solver.getUpperBoundLimits()
        return lb, ub

    def set_joint_limits(self, lower_bounds, upper_bounds):
        """
        Set joint limits for all the joints.

        :arg list lower_bounds: List of float of the lower bound limits for
            all joints.
        :arg list upper_bounds: List of float of the upper bound limits for
            all joints.
        """
        if len(lower_bounds) != self.number_of_joints:
            raise Exception("lower_bounds array size mismatch, it's size %i, should be %i" % (
                len(lower_bounds),
                self.number_of_joints))

        if len(upper_bounds) != self.number_of_joints:
            raise Exception("upper_bounds array size mismatch, it's size %i, should be %i" % (
                len(upper_bounds),
                self.number_of_joints))
        self._ik_solver.setKDLLimits(lower_bounds, upper_bounds)
