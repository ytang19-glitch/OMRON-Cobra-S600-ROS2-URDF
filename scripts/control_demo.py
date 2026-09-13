#!/usr/bin/env python3

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4']


class ControlDemo(Node):
    def __init__(self):
        super().__init__('cobra_control_demo')

        self.declare_parameter('mode', 'pd')
        self.declare_parameter('move_time', 5.0)
        self.declare_parameter('hold_time', 1.0)
        self.declare_parameter('target', [0.6, -0.8, 0.08, 0.6])
        self.declare_parameter('swing_amplitude', 0.8)
        self.declare_parameter('swing_period', 6.0)
        self.declare_parameter('kp', [40.0, 40.0, 220.0, 2.0])
        self.declare_parameter('kd', [12.0, 12.0, 35.0, 0.5])

        # Sliding/adaptive controller parameters.
        self.declare_parameter('smc_lambda', [5.0, 5.0, 8.0, 4.0])
        self.declare_parameter('smc_gain', [7.0, 7.0, 25.0, 1.0])
        self.declare_parameter('boundary_layer', 0.05)
        self.declare_parameter('adaptive_gamma', [2.0, 2.0, 5.0, 0.5])
        self.declare_parameter('adaptive_sigma', 0.10)
        self.declare_parameter('adaptive_gain_max', [40.0, 40.0, 120.0, 8.0])

        self.mode = str(self.get_parameter('mode').value).lower()
        self.move_time = float(self.get_parameter('move_time').value)
        self.hold_time = float(self.get_parameter('hold_time').value)
        self.target = [float(x) for x in self.get_parameter('target').value]
        self.swing_amplitude = min(
            abs(float(self.get_parameter('swing_amplitude').value)), 1.5)
        self.swing_period = float(self.get_parameter('swing_period').value)
        self.kp = [float(x) for x in self.get_parameter('kp').value]
        self.kd = [float(x) for x in self.get_parameter('kd').value]
        self.smc_lambda = [float(x) for x in self.get_parameter('smc_lambda').value]
        self.smc_gain = [float(x) for x in self.get_parameter('smc_gain').value]
        self.boundary_layer = max(
            1e-4, float(self.get_parameter('boundary_layer').value))
        self.adaptive_gamma = [
            float(x) for x in self.get_parameter('adaptive_gamma').value]
        self.adaptive_sigma = max(
            0.0, float(self.get_parameter('adaptive_sigma').value))
        self.adaptive_gain_max = [
            float(x) for x in self.get_parameter('adaptive_gain_max').value]

        aliases = {'linear': 'pd', 'nonlinear': 'computed_torque'}
        self.mode = aliases.get(self.mode, self.mode)
        valid_modes = ('pd', 'computed_torque', 'sliding_mode', 'adaptive')
        if self.mode not in valid_modes:
            raise ValueError(f'mode must be one of {valid_modes}')
        if self.swing_period <= 0.0:
            raise ValueError('swing_period must be positive')

        self.command_pub = self.create_publisher(
            Float64MultiArray, '/effort_controller/commands', 10)
        self.desired_pub = self.create_publisher(
            JointState, '/control_demo/desired', 10)
        self.error_pub = self.create_publisher(
            JointState, '/control_demo/error', 10)
        self.create_subscription(
            JointState, '/joint_states', self.joint_state_cb, 20)

        self.q = None
        self.qd = None
        self.q0 = None
        self.start_time = None
        self.last_control_time = None
        self.adaptive_gain = [0.0] * 4
        self.create_timer(0.004, self.control_step)

        self.get_logger().info(
            f'Starting {self.mode} controller: '
            f'Joint 1 amplitude={self.swing_amplitude:.3f} rad, '
            f'period={self.swing_period:.3f} s')

    def joint_state_cb(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        vel = dict(zip(msg.name, msg.velocity))
        if not all(j in pos for j in JOINTS):
            return
        self.q = [float(pos[j]) for j in JOINTS]
        self.qd = [float(vel.get(j, 0.0)) for j in JOINTS]
        if self.q0 is None:
            self.q0 = list(self.q)
            self.start_time = self.get_clock().now()
            self.last_control_time = self.start_time
            self.get_logger().info(
                f'Captured initial configuration: {self.q0}')

    @staticmethod
    def quintic(alpha: float) -> Tuple[float, float, float]:
        a = max(0.0, min(1.0, alpha))
        s = 10.0*a**3 - 15.0*a**4 + 6.0*a**5
        ds = 30.0*a**2 - 60.0*a**3 + 30.0*a**4
        dds = 60.0*a - 180.0*a**2 + 120.0*a**3
        return s, ds, dds

    def segment(self, q_start: List[float], q_goal: List[float], t: float):
        a = t / self.move_time
        s, ds_da, dds_da2 = self.quintic(a)
        ds_dt = ds_da / self.move_time
        dds_dt2 = dds_da2 / (self.move_time**2)
        delta = [q_goal[i] - q_start[i] for i in range(4)]
        q_ref = [q_start[i] + s*delta[i] for i in range(4)]
        dq_ref = [ds_dt*delta[i] for i in range(4)]
        ddq_ref = [dds_dt2*delta[i] for i in range(4)]
        return q_ref, dq_ref, ddq_ref

    def desired_trajectory(self, t: float):
        """Continuously swing Joint 1 while holding Joints 2-4 at startup."""
        omega = 2.0 * math.pi / self.swing_period
        q_ref = list(self.q0)
        dq_ref = [0.0] * 4
        ddq_ref = [0.0] * 4

        q_ref[0] = self.q0[0] + self.swing_amplitude * math.sin(omega * t)
        dq_ref[0] = self.swing_amplitude * omega * math.cos(omega * t)
        ddq_ref[0] = (
            -self.swing_amplitude * omega * omega * math.sin(omega * t))
        return q_ref, dq_ref, ddq_ref

    def tracking_errors(self, q_ref, dq_ref):
        e = [q_ref[i] - self.q[i] for i in range(4)]
        edot = [dq_ref[i] - self.qd[i] for i in range(4)]
        return e, edot

    def linear_pd(self, q_ref, dq_ref):
        """Joint-space PD: tau = Kp*e + Kd*e_dot."""
        e, edot = self.tracking_errors(q_ref, dq_ref)
        return [self.kp[i]*e[i] + self.kd[i]*edot[i] for i in range(4)]

    def scara_inverse_dynamics(self, virtual_acceleration):
        """Approximate inverse dynamics from committed URDF parameters."""
        q2 = self.q[1]
        dq1 = self.qd[0]
        dq2 = self.qd[1]

        l1 = 0.325
        lc1 = 0.156103
        lc2 = 0.063852
        m1 = 16.281256
        m2 = 8.424651
        i1 = 0.318069
        i2 = 0.114892
        m3 = 0.191579
        i4 = 0.000002
        g0 = 9.81

        c2 = math.cos(q2)
        s2 = math.sin(q2)
        m11 = i1 + i2 + m1*lc1**2 + m2*(
            l1**2 + lc2**2 + 2.0*l1*lc2*c2)
        m12 = i2 + m2*(lc2**2 + l1*lc2*c2)
        m22 = i2 + m2*lc2**2

        h = m2*l1*lc2*s2
        cqd1 = -h*(2.0*dq1*dq2 + dq2*dq2)
        cqd2 = h*dq1*dq1

        v = virtual_acceleration
        tau1 = m11*v[0] + m12*v[1] + cqd1
        tau2 = m12*v[0] + m22*v[1] + cqd2
        tau3 = m3*v[2] - m3*g0
        tau4 = i4*v[3]
        return [tau1, tau2, tau3, tau4]

    def nonlinear_computed_torque(self, q_ref, dq_ref, ddq_ref):
        """Computed torque: tau=M(q)v+C(q,dq)dq+g(q)."""
        e, edot = self.tracking_errors(q_ref, dq_ref)
        v = [
            ddq_ref[i] + self.kd[i]*edot[i] + self.kp[i]*e[i]
            for i in range(4)
        ]
        return self.scara_inverse_dynamics(v)

    def sliding_mode_control(self, q_ref, dq_ref, ddq_ref):
        """Model-based sliding-mode control with a tanh boundary layer."""
        e, edot = self.tracking_errors(q_ref, dq_ref)
        s = [edot[i] + self.smc_lambda[i]*e[i] for i in range(4)]
        v = [
            ddq_ref[i]
            + self.smc_lambda[i]*edot[i]
            + self.smc_gain[i]*math.tanh(s[i] / self.boundary_layer)
            for i in range(4)
        ]
        return self.scara_inverse_dynamics(v)

    def adaptive_control(self, q_ref, dq_ref, ddq_ref, dt):
        """Adaptive-gain robust tracking controller.

        k_hat_dot = gamma*|s| - sigma*k_hat. This adapts the robust gain
        online; it is not a full rigid-body parameter estimator.
        """
        e, edot = self.tracking_errors(q_ref, dq_ref)
        s = [edot[i] + self.smc_lambda[i]*e[i] for i in range(4)]

        for i in range(4):
            gain_dot = (
                self.adaptive_gamma[i]*abs(s[i])
                - self.adaptive_sigma*self.adaptive_gain[i])
            self.adaptive_gain[i] += dt*gain_dot
            self.adaptive_gain[i] = max(
                0.0,
                min(self.adaptive_gain_max[i], self.adaptive_gain[i]))

        v = [
            ddq_ref[i]
            + self.smc_lambda[i]*edot[i]
            + self.adaptive_gain[i]*math.tanh(
                s[i] / self.boundary_layer)
            for i in range(4)
        ]
        return self.scara_inverse_dynamics(v)

    @staticmethod
    def clamp_effort(tau):
        limits = [100.0, 100.0, 500.0, 20.0]
        return [
            max(-limits[i], min(limits[i], tau[i])) for i in range(4)]

    def publish_diagnostics(self, q_ref, dq_ref):
        stamp = self.get_clock().now().to_msg()

        desired = JointState()
        desired.header.stamp = stamp
        desired.name = JOINTS
        desired.position = q_ref
        desired.velocity = dq_ref
        self.desired_pub.publish(desired)

        error = JointState()
        error.header.stamp = stamp
        error.name = JOINTS
        error.position = [q_ref[i] - self.q[i] for i in range(4)]
        error.velocity = [dq_ref[i] - self.qd[i] for i in range(4)]
        self.error_pub.publish(error)

    def control_step(self):
        if self.q is None or self.qd is None or self.start_time is None:
            return

        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9
        dt = 0.004
        if self.last_control_time is not None:
            dt = max(
                1e-4,
                min(0.02, (now - self.last_control_time).nanoseconds * 1e-9))
        self.last_control_time = now

        q_ref, dq_ref, ddq_ref = self.desired_trajectory(elapsed)

        if self.mode == 'pd':
            tau = self.linear_pd(q_ref, dq_ref)
        elif self.mode == 'computed_torque':
            tau = self.nonlinear_computed_torque(q_ref, dq_ref, ddq_ref)
        elif self.mode == 'sliding_mode':
            tau = self.sliding_mode_control(q_ref, dq_ref, ddq_ref)
        else:
            tau = self.adaptive_control(q_ref, dq_ref, ddq_ref, dt)

        tau = self.clamp_effort(tau)
        msg = Float64MultiArray()
        msg.data = tau
        self.command_pub.publish(msg)
        self.publish_diagnostics(q_ref, dq_ref)


def main(args=None):
    rclpy.init(args=args)
    node = ControlDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        zero = Float64MultiArray()
        zero.data = [0.0, 0.0, 0.0, 0.0]
        node.command_pub.publish(zero)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
