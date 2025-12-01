#!/usr/bin/env python3
"""
Node 4: Manager Controller
Coordinates switching between swing-up and LQR modes
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String, Float64
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

class ManagerNode(Node):
    def __init__(self):
        super().__init__('manager_node')
        
        # Switching thresholds (from YAML)
        self.declare_parameter('switch_to_lqr_angle', 0.5)  # From YAML
        self.declare_parameter('switch_to_lqr_velocity', 4.0)  # From YAML
        self.declare_parameter('stabilization_time', 0.15)  # From YAML
        self.declare_parameter('fall_angle_threshold', 0.8)  # From YAML
        self.declare_parameter('control_frequency', 100.0)
        
        self.switch_angle = self.get_parameter('switch_to_lqr_angle').value
        self.switch_velocity = self.get_parameter('switch_to_lqr_velocity').value
        self.stabilization_time = self.get_parameter('stabilization_time').value
        self.fall_angle = self.get_parameter('fall_angle_threshold').value
        control_freq = self.get_parameter('control_frequency').value
        
        # State tracking
        self.controller_mode = 'SWING_UP'
        self.stabilization_counter = 0
        self.stabilization_samples = int(self.stabilization_time * control_freq)
        
        # Current state (filtered for switching logic)
        self.current_state = np.zeros(4)
        self.state_received = False
        
        self.filter_theta_dot = LowPassFilter(alpha=0.3)
        self.filter_alpha_dot = LowPassFilter(alpha=0.3)
        
        # Control efforts from other nodes
        self.current_swingup_effort = 0.0
        self.current_lqr_effort = 0.0
        self.current_disturbance_effort = 0.0
        
        # Setpoint tracking
        self.theta_setpoint = 0.0
        
        # Publishers
        self.effort_pub = self.create_publisher(
            Float64MultiArray, '/effort_controller/commands', 10
        )
        self.mode_pub = self.create_publisher(String, '/controller_mode', 10)
        self.setpoint_pub = self.create_publisher(Float64, '/control/theta_setpoint', 10)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.swingup_sub = self.create_subscription(
            Float64, '/control/swingup_effort', self.swingup_callback, 10
        )
        self.lqr_sub = self.create_subscription(
            Float64, '/control/lqr_effort', self.lqr_callback, 10
        )
        self.disturbance_sub = self.create_subscription(
            Float64, '/control/disturbance_effort', self.disturbance_callback, 10
        )
        
        self.control_frequency = control_freq
        self.control_period = 1.0 / self.control_frequency
        
        self.get_logger().info('='*70)
        self.get_logger().info('🚀 MANAGER NODE - DISTRIBUTED CONTROLLER')
        self.get_logger().info('='*70)
        self.get_logger().info(f'Switch angle: {self.switch_angle} rad')
        self.get_logger().info(f'Switch velocity: {self.switch_velocity} rad/s')
        self.get_logger().info('='*70)
    
    def swingup_callback(self, msg):
        self.current_swingup_effort = msg.data
    
    def lqr_callback(self, msg):
        self.current_lqr_effort = msg.data
    
    def disturbance_callback(self, msg):
        self.current_disturbance_effort = msg.data
    
    def joint_state_callback(self, msg):
        try:
            name_map = {name: i for i, name in enumerate(msg.name)}
            
            if 'revolute_joint' in name_map and 'first_pendulum_joint' in name_map:
                idx_theta = name_map['revolute_joint']
                idx_alpha = name_map['first_pendulum_joint']
                
                # Positions
                self.current_state[0] = self._wrap_to_pi(msg.position[idx_theta])
                self.current_state[1] = self._wrap_to_pi(msg.position[idx_alpha])
                
                # Filtered velocities
                raw_theta_dot = msg.velocity[idx_theta] if len(msg.velocity) > idx_theta else 0.0
                raw_alpha_dot = msg.velocity[idx_alpha] if len(msg.velocity) > idx_alpha else 0.0
                
                self.current_state[2] = self.filter_theta_dot.update(raw_theta_dot)
                self.current_state[3] = self.filter_alpha_dot.update(raw_alpha_dot)
                
                self.state_received = True
        except Exception as e:
            self.get_logger().error(f'Joint state callback error: {e}')
    
    def _wrap_to_pi(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))
    
    def check_switch_to_lqr(self, state):
        _, alpha, _, alpha_dot = state
        alpha_dist = abs(self._wrap_to_pi(alpha - np.pi))
        
        angle_ok = alpha_dist < self.switch_angle
        velocity_ok = abs(alpha_dot) < self.switch_velocity
        
        return angle_ok and velocity_ok
    
    def check_switch_to_swingup(self, state):
        _, alpha, _, _ = state
        alpha_dist = abs(self._wrap_to_pi(alpha - np.pi))
        return alpha_dist > self.fall_angle
    
    def control_loop(self):
        if not self.state_received:
            return
        
        # State machine logic
        if self.controller_mode == 'SWING_UP':
            if self.check_switch_to_lqr(self.current_state):
                self.stabilization_counter += 1
                if self.stabilization_counter >= self.stabilization_samples:
                    # SWITCH TO STABILIZE
                    self.controller_mode = 'STABILIZE'
                    self.stabilization_counter = 0
                    
                    # Capture theta setpoint
                    self.theta_setpoint = self.current_state[0]
                    
                    # Publish setpoint to LQR node
                    setpoint_msg = Float64()
                    setpoint_msg.data = self.theta_setpoint
                    self.setpoint_pub.publish(setpoint_msg)
                    
                    self.get_logger().info('='*70)
                    self.get_logger().info('✓✓✓ SWITCHED TO LQR!')
                    self.get_logger().info(f'📍 θ setpoint: {self.theta_setpoint:.4f} rad ({np.degrees(self.theta_setpoint):.1f}°)')
                    self.get_logger().info('='*70)
            else:
                self.stabilization_counter = 0
            
            # Use swing-up effort
            tau_arm = self.current_swingup_effort
            tau_pendulum = 0.0
        
        else:  # STABILIZE mode
            if self.check_switch_to_swingup(self.current_state):
                self.controller_mode = 'SWING_UP'
                self.get_logger().warn('⚠  FALLEN! Restarting swing-up')
            
            # Use LQR effort + disturbance
            tau_arm = self.current_lqr_effort
            tau_pendulum = self.current_disturbance_effort
        
        # Publish efforts
        effort_msg = Float64MultiArray()
        effort_msg.data = [float(tau_arm), float(tau_pendulum)]
        self.effort_pub.publish(effort_msg)
        
        # Publish mode
        mode_msg = String()
        mode_msg.data = self.controller_mode
        self.mode_pub.publish(mode_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ManagerNode()
    
    node.get_logger().info('Starting control loop...')
    
    try:
        while rclpy.ok():
            start = time.time()
            
            # Spin once to process callbacks
            rclpy.spin_once(node, timeout_sec=0.0)
            
            # Execute control loop
            node.control_loop()
            
            # Maintain control frequency
            elapsed = time.time() - start
            if elapsed < node.control_period:
                time.sleep(node.control_period - elapsed)
                
    except KeyboardInterrupt:
        pass
    finally:
        # Send zero torques on shutdown
        zero_msg = Float64MultiArray()
        zero_msg.data = [0.0, 0.0]
        node.effort_pub.publish(zero_msg)
        
        node.get_logger().info('\n' + '='*70)
        node.get_logger().info('✓ Manager shutting down')
        node.get_logger().info('='*70)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()