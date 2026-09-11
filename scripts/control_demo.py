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

        self.declare_parameter('mode', 'linear')
        self.declare_parameter('move_time', 5.0)
        self.declare_parameter('hold_time', 1.0)
        self.declare_parameter('target', [0.6, -0.8, 0.08, 0.6])
        self.declare_parameter('kp', [40.0, 40.0, 220.0, 2.0])
        self.declare_parameter('kd', [12.0, 12.0, 35.0, 0.5])

        self.mode = self.get_parameter('mode').value
        self.move_time = float(self.get_parameter('move_time').value)
        self.hold_time = float(self.get_parameter('hold_time').value)
        self.target = [float(x) for x in self.get_parameter('target').value]
        self.kp = [float(x) for x in self.get_parameter('kp').value]
        self.kd = [float(x) for x in self.get_parameter('kd').value]

        if self.mode not in ('linear', 'nonlinear'):
            raise ValueError("mode must be 'linear' or 'nonlinear'")

        self.command_pub = self.create_publisher(
            Float64MultiArray, '/effort_controller/commands', 10)
        self.desired_pub = self.create_publisher(
            JointState, '/control_demo/desired', 10)
        self.error_pub = self.create_publisher(
            JointState, '/control_demo/error', 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 20)

        self.q = None
        self.qd = None
        self.q0 = None
        self.start_time = None
        self.create_timer(0.004, self.control_step)  # 250 Hz

        self.get_logger().info(
            f'Starting {self.mode} control demo: home -> target -> home')

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
            self.get_logger().info(f'Captured initial configuration: {self.q0}')

    @staticmethod
    def quintic(alpha: float) -> Tuple[float, float, float]:
        """Return s, ds/dalpha, d2s/dalpha2 for quintic time scaling."""
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
        t1 = self.move_time
        t2 = t1 + self.hold_time
        t3 = t2 + self.move_time

        if t <= t1:
            return self.segment(self.q0, self.target, t)
        if t <= t2:
            return list(self.target), [0.0]*4, [0.0]*4
        if t <= t3:
            return self.segment(self.target, self.q0, t - t2)
        return list(self.q0), [0.0]*4, [0.0]*4

    def linear_pd(self, q_ref, dq_ref):
        """Linear joint-space PD torque controller: tau = Kp e + Kd e_dot."""
        return [
            self.kp[i]*(q_ref[i] - self.q[i])
            + self.kd[i]*(dq_ref[i] - self.qd[i])
            for i in range(4)
        ]

    def nonlinear_computed_torque(self, q_ref, dq_ref, ddq_ref):
        """Approximate SCARA computed-torque controller.

        tau = M(q) v + C(q,dq)dq + g(q)
        v = ddq_ref + Kd(dq_ref-dq) + Kp(q_ref-q)

        The first two axes use a planar 2R SCARA model derived from the current
        URDF dimensions/mass properties. Joint 3 uses vertical mass/gravity
        compensation, and joint 4 uses its axial inertia.
        """
        v = [
            ddq_ref[i]
            + self.kd[i]*(dq_ref[i] - self.qd[i])
            + self.kp[i]*(q_ref[i] - self.q[i])
            for i in range(4)
        ]

        q2 = self.q[1]
        dq1 = self.qd[0]
        dq2 = self.qd[1]

        # Approximate parameters from the committed URDF.
        l1 = 0.325
        lc1 = 0.156103
        lc2 = 0.063852
        m1 = 16.281256
        m2 = 8.424651
        i1 = 0.318069
        i2 = 0.114892
        m3 = 0.191579  # quill_slide + quill_link
        i4 = 0.000002
        g0 = 9.81

        c2 = math.cos(q2)
        s2 = math.sin(q2)

        m11 = i1 + i2 + m1*lc1**2 + m2*(l1**2 + lc2**2 + 2.0*l1*lc2*c2)
        m12 = i2 + m2*(lc2**2 + l1*lc2*c2)
        m22 = i2 + m2*lc2**2

        h = m2*l1*lc2*s2
        cqd1 = -h*(2.0*dq1*dq2 + dq2*dq2)
        cqd2 = h*dq1*dq1

        tau1 = m11*v[0] + m12*v[1] + cqd1
        tau2 = m12*v[0] + m22*v[1] + cqd2

        # joint_3 positive direction is downward (axis 0 0 -1), so gravity
        # generalized force in M*qdd + g(q) = tau is negative.
        tau3 = m3*v[2] - m3*g0
        tau4 = i4*v[3]

        return [tau1, tau2, tau3, tau4]

    @staticmethod
    def clamp_effort(tau):
        limits = [100.0, 100.0, 500.0, 20.0]
        return [max(-limits[i], min(limits[i], tau[i])) for i in range(4)]

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

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        q_ref, dq_ref, ddq_ref = self.desired_trajectory(elapsed)

        if self.mode == 'linear':
            tau = self.linear_pd(q_ref, dq_ref)
        else:
            tau = self.nonlinear_computed_torque(q_ref, dq_ref, ddq_ref)

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
