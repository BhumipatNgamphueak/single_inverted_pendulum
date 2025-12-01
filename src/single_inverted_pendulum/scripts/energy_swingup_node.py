#!/usr/bin/env python3
"""
Node 1: Energy-Based Swing-Up Controller
Constantly computes and publishes swing-up torque
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import time

class EnergySwingupNode(Node):
    def __init__(self):
        super().__init__('energy_swingup_node')
        
        # Physical parameters (from YAML)
        self.declare_parameter('g', 9.81)
        self.declare_parameter('M1', 0.190)
        self.declare_parameter('l1', 0.0878)  # From YAML
        self.declare_parameter('energy_gain', 2.55)
        self.declare_parameter('damping_gain', 0.05)  # From YAML
        self.declare_parameter('kick_torque', 3.0)  # From YAML
        self.declare_parameter('swing_up_max_torque', 15.0)  # From YAML
        self.declare_parameter('energy_regulation_threshold', 0.9)
        self.declare_parameter('pump_sign', -1.0)  # From YAML
        
        self.g = self.get_parameter('g').value
        self.M1 = self.get_parameter('M1').value
        self.l1 = self.get_parameter('l1').value
        self.energy_gain = self.get_parameter('energy_gain').value
        self.damping_gain = self.get_parameter('damping_gain').value
        self.kick_torque = self.get_parameter('kick_torque').value
        self.swing_up_max_torque = self.get_parameter('swing_up_max_torque').value
        self.energy_regulation_threshold = self.get_parameter('energy_regulation_threshold').value
        self.pump_sign = self.get_parameter('pump_sign').value
        
        self.E_desired = self.M1 * self.g * (2.0 * self.l1)
        
        # Control frequency
        self.control_frequency = 100.0
        self.control_period = 1.0 / self.control_frequency
        
        # Kick mechanism
        self.kick_applied = False
        self.kick_counter = 0
        self.kick_duration = 20  # ~0.2s at 100Hz
        
        # Current state
        self.current_state = np.zeros(4)
        self.state_received = False
        
        # Publishers and Subscribers
        self.effort_pub = self.create_publisher(Float64, '/control/swingup_effort', 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        self.get_logger().info('Energy Swing-Up Node Started')
        self.get_logger().info(f'E_desired: {self.E_desired:.4f} J')
    
    def joint_state_callback(self, msg):
        try:
            name_map = {name: i for i, name in enumerate(msg.name)}
            
            if 'revolute_joint' in name_map and 'first_pendulum_joint' in name_map:
                idx_theta = name_map['revolute_joint']
                idx_alpha = name_map['first_pendulum_joint']
                
                self.current_state[0] = self._wrap_to_pi(msg.position[idx_theta])
                self.current_state[1] = self._wrap_to_pi(msg.position[idx_alpha])
                self.current_state[2] = msg.velocity[idx_theta] if len(msg.velocity) > idx_theta else 0.0
                self.current_state[3] = msg.velocity[idx_alpha] if len(msg.velocity) > idx_alpha else 0.0
                
                self.state_received = True
                self._compute_and_publish()
        except Exception as e:
            self.get_logger().error(f'Callback error: {e}')
    
    def _wrap_to_pi(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))
    
    def _compute_energy(self, state):
        _, alpha, _, alpha_dot = state
        pe = self.M1 * self.g * self.l1 * (1.0 - np.cos(alpha))
        ke = 0.5 * self.M1 * (self.l1 * alpha_dot)**2
        return pe + ke
    
    def swing_up_control(self, state):
        theta, alpha, theta_dot, alpha_dot = state
        
        # Kick phase
        if not self.kick_applied:
            if self.kick_counter < self.kick_duration:
                self.kick_counter += 1
                return self.kick_torque
            else:
                self.kick_applied = True
                self.get_logger().info('✓ Kick done → Energy pumping')
        
        # Energy pumping with regulation
        E_current = self._compute_energy(state)
        E_error = self.E_desired - E_current
        energy_ratio = E_current / self.E_desired
        
        if energy_ratio > self.energy_regulation_threshold:
            regulation_factor = max(0.0, (1.0 - energy_ratio) / (1.0 - self.energy_regulation_threshold))
        else:
            regulation_factor = 1.0
        
        control_term = alpha_dot * np.cos(alpha)
        u_energy = self.pump_sign * self.energy_gain * E_error * control_term * regulation_factor
        u_damping = -self.damping_gain * theta_dot
        
        return u_energy + u_damping
    
    def _compute_and_publish(self):
        """Compute swing-up torque and publish"""
        if not self.state_received:
            return
        
        tau = self.swing_up_control(self.current_state)
        tau = np.clip(tau, -self.swing_up_max_torque, self.swing_up_max_torque)
        
        msg = Float64()
        msg.data = float(tau)
        self.effort_pub.publish(msg)
    
    def control_step(self):
        """Single control iteration"""
        # Process callbacks (joint_states)
        rclpy.spin_once(self, timeout_sec=0.0)
        
        # Compute and publish
        self._compute_and_publish()

def main(args=None):
    rclpy.init(args=args)
    node = EnergySwingupNode()
    
    node.get_logger().info('Starting energy swing-up loop...')
    
    try:
        while rclpy.ok():
            start = time.time()
            node.control_step()
            
            elapsed = time.time() - start
            if elapsed < node.control_period:
                time.sleep(node.control_period - elapsed)
                
    except KeyboardInterrupt:
        pass
    finally:
        # Send zero on shutdown
        msg = Float64()
        msg.data = 0.0
        node.effort_pub.publish(msg)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()