#!/usr/bin/env python3
"""
Node 2: LQR Stabilizer Controller
Computes stabilization torque for upright position
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import time

class LowPassFilter:
    def __init__(self, alpha=0.1):
        self.alpha = alpha
        self.value = 0.0
        self.initialized = False
    
    def update(self, new_val):
        if not self.initialized:
            self.value = new_val
            self.initialized = True
        else:
            self.value = self.alpha * new_val + (1.0 - self.alpha) * self.value
        return self.value

class LQRStabilizerNode(Node):
    def __init__(self):
        super().__init__('lqr_stabilizer_node')
        
        # LQR Gains (from YAML)
        self.declare_parameter('K_theta', 0.0)
        self.declare_parameter('K_alpha', -120.0)
        self.declare_parameter('K_theta_dot', 3.0)
        self.declare_parameter('K_alpha_dot', -20.0)
        self.declare_parameter('lqr_max_torque', 15.0)  # From YAML
        
        K_theta = self.get_parameter('K_theta').value
        K_alpha = self.get_parameter('K_alpha').value
        K_theta_dot = self.get_parameter('K_theta_dot').value
        K_alpha_dot = self.get_parameter('K_alpha_dot').value
        
        self.K_lqr = np.array([K_theta, K_alpha, K_theta_dot, K_alpha_dot])
        self.lqr_max_torque = self.get_parameter('lqr_max_torque').value
        
        # Theta setpoint (will be updated by manager)
        self.theta_setpoint = 0.0
        
        # Velocity filters
        self.filter_theta_dot = LowPassFilter(alpha=0.3)
        self.filter_alpha_dot = LowPassFilter(alpha=0.3)
        
        # Control frequency
        self.control_frequency = 100.0
        self.control_period = 1.0 / self.control_frequency
        
        # Current state (raw for LQR)
        self.current_state = np.zeros(4)
        self.state_received = False
        
        # Publishers and Subscribers
        self.effort_pub = self.create_publisher(Float64, '/control/lqr_effort', 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.setpoint_sub = self.create_subscription(
            Float64, '/control/theta_setpoint', self.setpoint_callback, 10
        )
        
        self.get_logger().info('LQR Stabilizer Node Started')
        self.get_logger().info(f'K matrix: {self.K_lqr}')
    
    def setpoint_callback(self, msg):
        self.theta_setpoint = msg.data
        self.get_logger().info(f'Updated θ setpoint: {self.theta_setpoint:.4f} rad')
    
    def joint_state_callback(self, msg):
        try:
            name_map = {name: i for i, name in enumerate(msg.name)}
            
            if 'revolute_joint' in name_map and 'first_pendulum_joint' in name_map:
                idx_theta = name_map['revolute_joint']
                idx_alpha = name_map['first_pendulum_joint']
                
                # Raw positions
                self.current_state[0] = self._wrap_to_pi(msg.position[idx_theta])
                self.current_state[1] = self._wrap_to_pi(msg.position[idx_alpha])
                
                # Filtered velocities
                raw_theta_dot = msg.velocity[idx_theta] if len(msg.velocity) > idx_theta else 0.0
                raw_alpha_dot = msg.velocity[idx_alpha] if len(msg.velocity) > idx_alpha else 0.0
                
                self.current_state[2] = self.filter_theta_dot.update(raw_theta_dot)
                self.current_state[3] = self.filter_alpha_dot.update(raw_alpha_dot)
                
                self.state_received = True
                self._compute_and_publish()
        except Exception as e:
            self.get_logger().error(f'Callback error: {e}')
    
    def _wrap_to_pi(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))
    
    def lqr_control(self, state):
        theta, alpha, theta_dot, alpha_dot = state
        
        theta_error = self._wrap_to_pi(theta - self.theta_setpoint)
        alpha_error = self._wrap_to_pi(alpha - np.pi)
        
        x_error = np.array([theta_error, alpha_error, theta_dot, alpha_dot])
        u = -self.K_lqr @ x_error
        
        return float(u)
    
    def _compute_and_publish(self):
        if not self.state_received:
            return
        
        tau = self.lqr_control(self.current_state)
        tau = np.clip(tau, -self.lqr_max_torque, self.lqr_max_torque)
        
        msg = Float64()
        msg.data = float(tau)
        self.effort_pub.publish(msg)
    
    def control_step(self):
        """Single control iteration"""
        # Process callbacks (joint_states, setpoint)
        rclpy.spin_once(self, timeout_sec=0.0)
        
        # Compute and publish
        self._compute_and_publish()

def main(args=None):
    rclpy.init(args=args)
    node = LQRStabilizerNode()
    
    node.get_logger().info('Starting LQR stabilizer loop...')
    
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