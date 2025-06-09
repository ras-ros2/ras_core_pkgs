#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from ras_interfaces.srv import ArucoPoses
import yaml
import os
import time
from pathlib import Path
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np
from tf_transformations import euler_from_quaternion
from ras_common.globals import RAS_CONFIGS_PATH

class ArucoExperimentManager(Node):
    def __init__(self):
        super().__init__('aruco_experiment_manager')
        
        # Set up experiment directory
        self.experiment_dir = os.path.join(RAS_CONFIGS_PATH, 'experiments')
        os.makedirs(self.experiment_dir, exist_ok=True)
        self.experiment_file = os.path.join(self.experiment_dir, 'aruco_experiment.yaml')
        
        # Dictionary to store marker poses (key: marker_id, value: pose)
        self.marker_poses = {}
        
        # Keep track of previously seen marker IDs to avoid regenerating files
        self.seen_marker_ids = set()
        
        # Create a service server for ArUco poses (instead of a client)
        self.callback_group = ReentrantCallbackGroup()
        self.aruco_service = self.create_service(
            ArucoPoses,
            "/aruco_poses",
            self.aruco_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info("ArucoExperimentManager initialized")
        self.get_logger().info(f"Waiting for ArUco marker data on /aruco_poses service...")
    
    def aruco_callback(self, request, response):
        """Callback for ArUco pose service"""
        try:
            self.get_logger().info("Received ArUco marker data")
            
            if hasattr(request, 'marker_id_list') and hasattr(request, 'poses'):
                marker_count = len(request.marker_id_list)
                self.get_logger().info(f"Received {marker_count} markers")
                
                # Print current known marker IDs
                self.get_logger().info(f"Currently known marker IDs: {self.seen_marker_ids}")
                
                # Check if there are any new marker IDs
                new_marker_detected = False
                new_markers = []
                
                # Update marker poses from the received data
                for marker_id, pose in zip(request.marker_id_list, request.poses):
                    self.get_logger().info(f"Processing marker with ID: {marker_id}")
                    if marker_id not in self.seen_marker_ids:
                        self.get_logger().info(f"NEW MARKER DETECTED with ID: {marker_id}")
                        new_marker_detected = True
                        new_markers.append(marker_id)
                        self.seen_marker_ids.add(marker_id)
                    else:
                        self.get_logger().info(f"Updating existing marker with ID: {marker_id}")
                    
                    self.marker_poses[marker_id] = pose
                    self.get_logger().info(f"Marker {marker_id} at position: [{pose.position.x}, {pose.position.y}, {pose.position.z}]")
                
                # Generate experiment file only if we have new markers detected
                if new_marker_detected and self.marker_poses:
                    self.get_logger().info(f"New markers detected: {new_markers}! Generating experiment with {len(self.marker_poses)} markers")
                    self.generate_experiment_file(create_timestamped_copy=True)
                else:
                    self.get_logger().info(f"No new markers detected. Known markers: {list(self.seen_marker_ids)}")
            else:
                self.get_logger().error("Request is missing marker_id_list or poses")
                
            # Always return success to acknowledge receipt
            response.response = True
            return response
                
        except Exception as e:
            self.get_logger().error(f"Error in aruco_callback: {str(e)}", exc_info=True)
            response.response = False
            return response
    
    def generate_experiment_file(self, create_timestamped_copy=False):
        """Generate experiment YAML file based on detected markers
        
        Args:
            create_timestamped_copy (bool): Whether to also create a timestamped copy of the file
        """
        self.get_logger().info("Generating experiment file...")
        # Sort marker IDs for consistent ordering
        sorted_marker_ids = sorted(self.marker_poses.keys())
        self.get_logger().info(f"Sorted marker IDs: {sorted_marker_ids}")
        
        # Create experiment dictionary
        experiment = {
            'Poses': {},
            'targets': [
                {'Move': 'home'},  # Start from home
            ]
        }
        
        # Add home position with valid angle values
        experiment['Poses']['home'] = {
            'x': 20.0, 'y': 0.0, 'z': 50.0,
            'roll': 3.14, 'pitch': 0.0, 'yaw': 0.0
        }
        
        # Stack position (fixed for all markers)
        stack_x = 20.0  # Changed from 0.0 to 20.0 for better reachability
        stack_y = 0.0
        stack_z_base = 15.0  # Base height for stacking
        stack_z_step = 5.0   # Height increment per object
        
        # Add poses and targets for each marker
        for i, marker_id in enumerate(sorted_marker_ids, 1):
            pose = self.marker_poses[marker_id]
            
            # Convert marker position to cm and round to 2 decimal places
            marker_x = round(float(pose.position.x * 100), 2)
            marker_y = round(float(pose.position.y * 100), 2)
            
            # Add approach position (above the marker)
            experiment['Poses'][f'approach_{i}'] = {
                'x': marker_x,
                'y': marker_y,
                'z': 25.0,  # Fixed approach height
                'roll': 3.14,
                'pitch': 0.0,
                'yaw': 0.0  # Fixed yaw value instead of using marker orientation
            }
            
            # Add pick position (at the marker)
            experiment['Poses'][f'pick_{i}'] = {
                'x': marker_x,
                'y': marker_y,
                'z': 9.0, # Fixed pick height
                'roll': 3.14,
                'pitch': 0.0,
                'yaw': 0.0  # Fixed yaw value instead of using marker orientation
            }
            
            # Add stack approach position (above the stack)
            experiment['Poses'][f'stack_approach_{i}'] = {
                'x': stack_x,
                'y': stack_y,
                'z': 25.0,  # Fixed approach height
                'roll': 3.14,
                'pitch': 0.0,
                'yaw': 0.0
            }
            
            # Add stack place position (on top of the stack)
            stack_z = stack_z_base + (i-1) * stack_z_step
            experiment['Poses'][f'stack_place_{i}'] = {
                'x': stack_x,
                'y': stack_y,
                'z': stack_z,  # Stack height increases with each object
                'roll': 3.14,
                'pitch': 0.0,
                'yaw': 0.0
            }
            
            # Add sequence of actions for this marker
            experiment['targets'].extend([
                {'Move': f'approach_{i}'},  # Move above the marker
                {'Pick': f'pick_{i}'},      # Pick the marker
                {'Move': f'stack_approach_{i}'},  # Move above the stack
                {'Place': f'stack_place_{i}'}     # Place on the stack
            ])
        
        # Return to home after all operations
        experiment['targets'].append({'Move': 'home'})
        
        # Write to file in RAS_CONFIGS_PATH/experiments
        with open(self.experiment_file, 'w') as f:
            yaml.dump(experiment, f, default_flow_style=False, sort_keys=False)
        
        self.get_logger().info(f"Updated experiment file at {self.experiment_file}")
        
        # Only create a timestamped copy if requested
        if create_timestamped_copy:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            history_file = os.path.join(self.experiment_dir, f'aruco_experiment_{timestamp}.yaml')
            with open(history_file, 'w') as f:
                yaml.dump(experiment, f, default_flow_style=False, sort_keys=False)
            self.get_logger().info(f"Created timestamped copy at {history_file}")

def main(args=None):
    rclpy.init(args=args)
    
    node = ArucoExperimentManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()