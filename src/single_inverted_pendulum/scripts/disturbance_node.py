#!/usr/bin/env python3
"""
Node 3: Disturbance Generator
Applies brief torque pulse after stabilization delay
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
import time

class DisturbanceNode(Node):
    def __init__(self):
        super().__init__('disturbance_node')
        
        # Disturbance parameters
        self.declare_parameter('disturbance_delay', 5.0)
        self.declare_parameter('disturbance_duration', 0.15)
        self.declare_parameter('disturbance_torque', 0.07)
        
        self.disturbance_delay = self.get_parameter('disturbance_delay').value
        self.disturbance_duration = self.get_parameter('disturbance_duration').value
        self.disturbance_torque = self.get_parameter('disturbance_torque').value
        
        # Control frequency
        self.control_frequency = 100.0
        self.control_period = 1.0 / self.control_frequency
        
        # State tracking
        self.stabilize_start_time = None
        self.disturbance_applied = False
        self.current_mode = 'SWING_UP'
        
        # Publishers and Subscribers
        self.effort_pub = self.create_publisher(Float64, '/control/disturbance_effort', 10)
        self.mode_sub = self.create_subscription(
            String, '/controller_mode', self.mode_callback, 10
        )
        
        self.get_logger().info('Disturbance Node Started')
        self.get_logger().info(f'Delay: {self.disturbance_delay}s, Duration: {self.disturbance_duration}s')
        self.get_logger().info(f'Torque: {self.disturbance_torque} Nm')
    
    def mode_callback(self, msg):
        new_mode = msg.data
        
        # Detect transition to STABILIZE
        if new_mode == 'STABILIZE' and self.current_mode != 'STABILIZE':
            self.stabilize_start_time = time.time()
            self.disturbance_applied = False
            self.get_logger().info(f'⏱️  Disturbance timer started ({self.disturbance_delay}s countdown)')
        
        # Reset on return to SWING_UP
        if new_mode == 'SWING_UP':
            self.stabilize_start_time = None
            self.disturbance_applied = False
        
        self.current_mode = new_mode
    
    def get_disturbance_torque(self):
        if self.current_mode != 'STABILIZE' or self.stabilize_start_time is None:
            return 0.0
        
        time_in_stabilize = time.time() - self.stabilize_start_time
        
        disturbance_start = self.disturbance_delay
        disturbance_end = self.disturbance_delay + self.disturbance_duration
        
        if disturbance_start <= time_in_stabilize < disturbance_end:
            if not self.disturbance_applied:
                self.disturbance_applied = True
                self.get_logger().warn('='*50)
                self.get_logger().warn(f'⚡ APPLYING DISTURBANCE: {self.disturbance_torque} Nm')
                self.get_logger().warn('='*50)
            return self.disturbance_torque
        
        elif time_in_stabilize >= disturbance_end and self.disturbance_applied:
            self.get_logger().info('✓ Disturbance complete')
            self.disturbance_applied = True  # Prevent re-logging
        
        return 0.0
    
    def publish_step(self):
        """Compute and publish disturbance torque"""
        tau_dist = self.get_disturbance_torque()
        
        msg = Float64()
        msg.data = float(tau_dist)
        self.effort_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DisturbanceNode()
    
    node.get_logger().info('Starting disturbance loop...')
    
    try:
        while rclpy.ok():
            start = time.time()
            
            # Spin once to process callbacks
            rclpy.spin_once(node, timeout_sec=0.0)
            
            # Publish disturbance effort
            node.publish_step()
            
            # Maintain control frequency
            elapsed = time.time() - start
            if elapsed < node.control_period:
                time.sleep(node.control_period - elapsed)
                
    except KeyboardInterrupt:
        pass
    finally:
        # Send zero on shutdown
        zero_msg = Float64()
        zero_msg.data = 0.0
        node.effort_pub.publish(zero_msg)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()