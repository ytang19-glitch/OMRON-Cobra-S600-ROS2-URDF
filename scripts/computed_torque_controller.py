#!/usr/bin/env python3
"""Educational computed-torque controller for the Cobra s600 Gazebo model.

The controller publishes generalized efforts in this order:
joint_1 torque, joint_2 torque, joint_3 force, joint_4 torque.
It is disabled by default so that launching Gazebo cannot command sudden motion.
"""

import math
from typing import Dict, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4']


class ComputedTorqueController(Node):
    def __init__(self) -> None:
        super().__init__('computed_torque_controller')

        self.declare_parameter('enabled', False)
        self.declare_parameter('desired_positions', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('desired_velocities', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('desired_accelerations', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('kp', [40.0, 40.0, 100.0, 30.0])
        self.declare_parameter('kd', [12.0, 12.0, 20.0, 6.0])
        self.declare_parameter('effort_limits', [40.0, 40.0, 100.0, 5.0])
        self.declare_parameter('control_rate', 250.0)

        # Initial model values derived from the current URDF. These are not a
        # manufacturer-validated actuator/dynamics model.
        self.declare_parameter('link_1_length', 0.325)
        self.declare_parameter('link_2_length', 0.275)
        self.declare_parameter('link_1_com', 0.156103)
        self.declare_parameter('link_2_com', 0.063852)
        self.declare_parameter('link_1_mass', 16.281256)
        self.declare_parameter('link_2_mass', 8.424651)
        self.declare_parameter('quill_mass', 0.191479)
        self.declare_parameter('link_1_izz', 0.318069)
        self.declare_parameter('link_2_izz', 0.114892)
        self.declare_parameter('joint_4_effective_inertia', 0.002)
        self.declare_parameter('gravity', 9.80665)

        self._positions: Optional[np.ndarray] = None
        self._velocities: Optional[np.ndarray] = None
        self._warned_disabled = False

        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)
        self._publisher = self.create_publisher(
            Float64MultiArray, '/effort_controller/commands', 10)

        rate = float(self.get_parameter('control_rate').value)
        if rate <= 0.0:
            raise ValueError('control_rate must be positive')
        self.create_timer(1.0 / rate, self._update)

        self.get_logger().info(
            'Computed-torque controller started DISABLED. Enable only after '
            'checking joint states and targets: ros2 param set '
            '/computed_torque_controller enabled true')

    def _joint_state_cb(self, msg: JointState) -> None:
        by_name: Dict[str, int] = {name: i for i, name in enumerate(msg.name)}
        if not all(name in by_name for name in JOINTS):
            return
        if len(msg.position) < len(msg.name) or len(msg.velocity) < len(msg.name):
            return
        self._positions = np.array([msg.position[by_name[name]] for name in JOINTS])
        self._velocities = np.array([msg.velocity[by_name[name]] for name in JOINTS])

    def _array_parameter(self, name: str) -> np.ndarray:
        value = np.asarray(self.get_parameter(name).value, dtype=float)
        if value.shape != (4,):
            raise ValueError(f'{name} must contain exactly four values')
        return value

    def _model_terms(self, q: np.ndarray, dq: np.ndarray):
        q2 = q[1]
        dq1, dq2 = dq[0], dq[1]

        l1 = float(self.get_parameter('link_1_length').value)
        l2 = float(self.get_parameter('link_2_length').value)
        r1 = float(self.get_parameter('link_1_com').value)
        r2 = float(self.get_parameter('link_2_com').value)
        m1 = float(self.get_parameter('link_1_mass').value)
        m2 = float(self.get_parameter('link_2_mass').value)
        mq = float(self.get_parameter('quill_mass').value)
        i1 = float(self.get_parameter('link_1_izz').value)
        i2 = float(self.get_parameter('link_2_izz').value)
        i4 = float(self.get_parameter('joint_4_effective_inertia').value)
        gravity = float(self.get_parameter('gravity').value)

        # Combine outer-link and quill masses into one planar equivalent.
        m2e = m2 + mq
        r2e = (m2 * r2 + mq * l2) / m2e
        i2e = i2 + m2 * (r2 - r2e) ** 2 + mq * (l2 - r2e) ** 2

        cosine = math.cos(q2)
        sine = math.sin(q2)
        coupling = m2e * l1 * r2e

        mass_matrix = np.zeros((4, 4))
        mass_matrix[0, 0] = (
            i1 + m1 * r1 ** 2 + i2e
            + m2e * (l1 ** 2 + r2e ** 2 + 2.0 * l1 * r2e * cosine))
        mass_matrix[0, 1] = i2e + m2e * (r2e ** 2 + l1 * r2e * cosine)
        mass_matrix[1, 0] = mass_matrix[0, 1]
        mass_matrix[1, 1] = i2e + m2e * r2e ** 2
        mass_matrix[2, 2] = mq
        mass_matrix[3, 3] = i4

        h = coupling * sine
        coriolis = np.array([
            -h * (2.0 * dq1 * dq2 + dq2 ** 2),
            h * dq1 ** 2,
            0.0,
            0.0,
        ])

        # joint_3 is positive downward, so compensation is an upward force.
        gravity_vector = np.array([0.0, 0.0, -mq * gravity, 0.0])
        return mass_matrix, coriolis, gravity_vector

    def _update(self) -> None:
        if self._positions is None or self._velocities is None:
            return
        if not bool(self.get_parameter('enabled').value):
            if not self._warned_disabled:
                self.get_logger().warn('Controller is disabled; no effort commands are being published.')
                self._warned_disabled = True
            return

        self._warned_disabled = False
        try:
            qd = self._array_parameter('desired_positions')
            dqd = self._array_parameter('desired_velocities')
            ddqd = self._array_parameter('desired_accelerations')
            kp = self._array_parameter('kp')
            kd = self._array_parameter('kd')
            limits = np.abs(self._array_parameter('effort_limits'))
        except ValueError as error:
            self.get_logger().error(str(error))
            return

        q = self._positions
        dq = self._velocities
        virtual_acceleration = ddqd + kd * (dqd - dq) + kp * (qd - q)
        mass_matrix, coriolis, gravity_vector = self._model_terms(q, dq)
        effort = mass_matrix @ virtual_acceleration + coriolis + gravity_vector
        effort = np.clip(effort, -limits, limits)

        message = Float64MultiArray()
        message.data = effort.tolist()
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ComputedTorqueController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
