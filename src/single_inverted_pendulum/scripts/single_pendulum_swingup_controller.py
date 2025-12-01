#!/usr/bin/env python3
"""
SINGLE PENDULUM - ENERGY REGULATED SWING-UP + BRIEF DISTURBANCE
===============================================================
LQR uses revolute position at stabilization as setpoint
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
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

class SinglePendulumLoopController(Node):
    
    def __init__(self):
        super().__init__('swingup_controller')
        
        self.control_frequency = 100.0
        self.control_period = 1.0 / self.control_frequency
        
        # Physical parameters
        self.g = 9.81
        self.M1 = 0.190
        self.l1 = 0.072
        self.l_arm = 0.157
        
        # ============================================================
        # SWING-UP PARAMETERS
        # ============================================================
        self.energy_gain = 5.00        
        self.damping_gain = 0.11
        self.pump_sign = -1.0
        self.energy_regulation_threshold = 0.9
        self.swing_up_max_torque = 10.0
        self.kick_torque = 5.0
        
        # ============================================================
        # LQR PARAMETERS
        # ============================================================
        self.K_theta = 2.40000 
        self.K_alpha = 9.24337       
        self.K_theta_dot = 0.96104
        self.K_alpha_dot = -0.44720
        self.lqr_max_torque = 5.0
        
        # ============================================================
        # EXTERNAL DISTURBANCE PARAMETERS
        # ============================================================
        self.disturbance_delay = 5.0           # Wait 5s after stabilization
        self.disturbance_duration = 0.15       # Brief pulse: 150ms
        self.disturbance_torque = 0.0          # Torque magnitude (Nm)
        
        # ============================================================
        # SWITCHING THRESHOLDS
        # ============================================================
        self.switch_angle = 0.20
        self.switch_velocity = 2.0
        self.stabilization_time = 0.05
        self.fall_angle = 0.7
        
        # Filtering
        self.filter_theta_dot = LowPassFilter(alpha=0.3)
        self.filter_alpha_dot = LowPassFilter(alpha=0.3)
        
        # State variables
        self.current_state = np.zeros(4)
        self.current_state_raw = np.zeros(4)
        self.state_received = False
        self.controller_mode = 'SWING_UP'
        
        # ============================================================
        # SETPOINT TRACKING - NEW
        # ============================================================
        self.theta_setpoint = 0.0  # Captured when entering STABILIZE mode
        
        # Kick mechanism
        self.kick_counter = 0
        self.kick_duration = int(0.2 * self.control_frequency)
        self.kick_applied = False
        
        # Disturbance tracking
        self.stabilize_start_time = None
        self.disturbance_applied = False
        
        self.K_lqr = np.array([
            self.K_theta, self.K_alpha, 
            self.K_theta_dot, self.K_alpha_dot
        ])
        
        self.E_desired = self.M1 * self.g * (2.0 * self.l1)
        
        self.stabilization_counter = 0
        self.stabilization_samples = int(self.stabilization_time * self.control_frequency)
        
        self.joint_names = ['revolute_joint', 'first_pendulum_joint']
        
        self.effort_pub = self.create_publisher(
            Float64MultiArray, '/effort_controller/commands', 10
        )
        self.mode_pub = self.create_publisher(String, '/controller_mode', 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        self._log_startup()
    
    def _log_startup(self):
        self.get_logger().info('='*70)
        self.get_logger().info('🚀 CONTROLLER WITH SETPOINT TRACKING')
        self.get_logger().info('='*70)
        self.get_logger().info(f'DISTURBANCE: {self.disturbance_torque} Nm')
        self.get_logger().info(f'  Delay: {self.disturbance_delay}s after stabilization')
        self.get_logger().info(f'  Duration: {self.disturbance_duration}s (brief pulse)')
        self.get_logger().info('')
        self.get_logger().info('LQR SETPOINT: θ captured at stabilization')
        self.get_logger().info('='*70)

    def joint_state_callback(self, msg):
        try:
            state_filtered = np.zeros(4)
            state_raw = np.zeros(4)
            name_map = {name: i for i, name in enumerate(msg.name)}
            
            raw_theta_dot = 0.0
            raw_alpha_dot = 0.0
            
            if self.joint_names[0] in name_map:
                idx = name_map[self.joint_names[0]]
                state_filtered[0] = self._wrap_to_pi(msg.position[idx])
                state_raw[0] = state_filtered[0]
                raw_theta_dot = msg.velocity[idx] if len(msg.velocity) > idx else 0.0
            
            if self.joint_names[1] in name_map:
                idx = name_map[self.joint_names[1]]
                state_filtered[1] = self._wrap_to_pi(msg.position[idx])
                state_raw[1] = state_filtered[1]
                raw_alpha_dot = msg.velocity[idx] if len(msg.velocity) > idx else 0.0
            
            state_raw[2] = raw_theta_dot
            state_raw[3] = raw_alpha_dot
            
            state_filtered[2] = self.filter_theta_dot.update(raw_theta_dot)
            state_filtered[3] = self.filter_alpha_dot.update(raw_alpha_dot)
            
            self.current_state = state_filtered
            self.current_state_raw = state_raw
            self.state_received = True
                
        except Exception as e:
            self.get_logger().error(f'Callback error: {e}')
    
    def _wrap_to_pi(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))
    
    def _compute_energy(self, state):
        """Compute total energy"""
        _, alpha, theta_dot, alpha_dot = state
        
        pe = self.M1 * self.g * self.l1 * (1.0 - np.cos(alpha))
        ke = 0.5 * self.M1 * (self.l1 * alpha_dot)**2
        
        return pe + ke
    
    def get_disturbance_torque(self):
        """Calculate disturbance torque based on timing"""
        if self.controller_mode != 'STABILIZE':
            return 0.0
        
        if self.stabilize_start_time is None:
            return 0.0
        
        time_in_stabilize = time.time() - self.stabilize_start_time
        
        # Check if in disturbance window
        disturbance_start = self.disturbance_delay
        disturbance_end = self.disturbance_delay + self.disturbance_duration
        
        if disturbance_start <= time_in_stabilize < disturbance_end:
            if not self.disturbance_applied:
                self.disturbance_applied = True
                self.get_logger().warn('='*70)
                self.get_logger().warn(f'⚡ APPLYING DISTURBANCE: {self.disturbance_torque} Nm')
                self.get_logger().warn('='*70)
            return self.disturbance_torque
        elif time_in_stabilize >= disturbance_end and self.disturbance_applied:
            self.get_logger().info('='*70)
            self.get_logger().info('✓ Disturbance complete')
            self.get_logger().info('='*70)
            # Prevent logging again
            self.disturbance_applied = True
        
        return 0.0
    
    def swing_up_control(self, state):
        """Energy-based swing-up with regulation"""
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
        
        if int(time.time() * 2) % 2 == 0:
            self.get_logger().info(
                f'E: {E_current:.4f}/{self.E_desired:.4f} ({energy_ratio*100:.1f}%), '
                f'Reg: {regulation_factor:.2f}, α̇: {alpha_dot:.1f}',
                throttle_duration_sec=0.5
            )
        
        return u_energy + u_damping
    
    def lqr_control(self, state):
        """LQR stabilization with captured setpoint"""
        theta, alpha, theta_dot, alpha_dot = state
        
        # Use captured theta_setpoint instead of 0
        theta_error = self._wrap_to_pi(theta - self.theta_setpoint)
        alpha_error = self._wrap_to_pi(alpha - np.pi)
        
        x_error = np.array([theta_error, alpha_error, theta_dot, alpha_dot])
        
        u = -self.K_lqr @ x_error
        
        return float(u)
    
    def check_switch_to_lqr(self, state):
        """Check if ready for LQR"""
        _, alpha, _, alpha_dot = state
        
        alpha_dist = abs(self._wrap_to_pi(alpha - np.pi))
        
        angle_ok = alpha_dist < self.switch_angle
        velocity_ok = abs(alpha_dot) < self.switch_velocity
        
        return angle_ok and velocity_ok
    
    def check_switch_to_swingup(self, state):
        """Check if fallen"""
        _, alpha, _, _ = state
        alpha_dist = abs(self._wrap_to_pi(alpha - np.pi))
        return alpha_dist > self.fall_angle
    
    def control_step(self):
        """Single control iteration"""
        
        rclpy.spin_once(self, timeout_sec=0.0)
        
        if not self.state_received:
            return
        
        # Use filtered for swing-up, raw for LQR
        if self.controller_mode == 'SWING_UP':
            x = self.current_state
        else:
            x = self.current_state_raw
        
        # State machine
        if self.controller_mode == 'SWING_UP':
            if self.check_switch_to_lqr(self.current_state):
                self.stabilization_counter += 1
                if self.stabilization_counter >= self.stabilization_samples:
                    self.controller_mode = 'STABILIZE'
                    self.stabilization_counter = 0
                    
                    # ============================================================
                    # CAPTURE THETA SETPOINT - NEW
                    # ============================================================
                    self.theta_setpoint = self.current_state[0]
                    
                    self.stabilize_start_time = time.time()
                    self.disturbance_applied = False
                    self.get_logger().info('='*70)
                    self.get_logger().info('✓✓✓ SWITCHED TO LQR!')
                    self.get_logger().info(f'📍 θ setpoint: {self.theta_setpoint:.4f} rad ({np.degrees(self.theta_setpoint):.1f}°)')
                    self.get_logger().info(f'⏱️  Disturbance in {self.disturbance_delay}s...')
                    self.get_logger().info('='*70)
            else:
                self.stabilization_counter = 0
            
            tau_arm = self.swing_up_control(x)
            tau_pendulum = 0.0
            max_t = self.swing_up_max_torque
            
        else:  # STABILIZE
            if self.check_switch_to_swingup(x):
                self.controller_mode = 'SWING_UP'
                self.kick_applied = False
                self.kick_counter = 0
                self.stabilize_start_time = None
                self.get_logger().warn('⚠ FALLEN! Restarting')
            
            tau_arm = self.lqr_control(x)
            
            # APPLY DISTURBANCE TO PENDULUM JOINT
            tau_pendulum = self.get_disturbance_torque()
            
            max_t = self.lqr_max_torque
        
        # Saturate arm torque
        tau_arm = np.clip(tau_arm, -max_t, max_t)
        
        # Publish [arm_torque, pendulum_torque]
        msg = Float64MultiArray()
        msg.data = [float(tau_arm), float(tau_pendulum)]
        self.effort_pub.publish(msg)
        
        mode_msg = String()
        mode_msg.data = self.controller_mode
        self.mode_pub.publish(mode_msg)

    def run(self):
        """Main control loop"""
        self.get_logger().info('Starting control loop...')
        self.get_logger().info('='*70)
        
        try:
            while rclpy.ok():
                start = time.time()
                self.control_step()
                elapsed = time.time() - start
                if elapsed < self.control_period:
                    time.sleep(self.control_period - elapsed)
                    
        except KeyboardInterrupt:
            self.get_logger().info('\n' + '='*70)
            self.get_logger().info('Shutting down...')
            self.get_logger().info('='*70)
        finally:
            msg = Float64MultiArray()
            msg.data = [0.0, 0.0]
            self.effort_pub.publish(msg)
            time.sleep(0.1)

def main(args=None):
    print("="*70)
    print("🚀 CONTROLLER WITH SETPOINT TRACKING")
    print("="*70)
    
    rclpy.init(args=args)
    controller = SinglePendulumLoopController()
    controller.run()
    controller.destroy_node()
    rclpy.shutdown()
    
    print("\n" + "="*70)
    print("✓ Controller stopped")
    print("="*70)

if __name__ == '__main__':
    main()