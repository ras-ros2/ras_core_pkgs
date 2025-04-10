#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from ras_interfaces.srv import ArucoPoses, SpawnSim, MoveSim
from ras_common.config.loaders.lab_setup import LabSetup as LabLoader
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import yaml

class SpawnIgn(Node):
    def __init__(self):
        super().__init__("spawn_ign_node")
        self.get_logger().info("IGN SPAWN MANAGER NODE STARTED")
        LabLoader.init()
        
        self.robot_pose = LabLoader.robot_pose

        # self.spawn_pose = self.create_publisher(Pose, "/spawn_model", 10)
        self.spawn_client = self.create_client(SpawnSim, "/spawn_object")
        self.move_client = self.create_client(MoveSim, "/move_object")
        self.despawn_pose = self.create_publisher(String, "/despawn_model", 10)

        self.create_service(ArucoPoses, "/aruco_poses", self.aruco_spawner)

        self.spawned_object = []
        
        self.robot_rot_eulers = euler_from_quaternion((self.robot_pose.orientation.x, self.robot_pose.orientation.y, self.robot_pose.orientation.z, self.robot_pose.orientation.w))
    
    def aruco_spawner(self, req, resp):
        for i in range(len(req.marker_id_list)):
            if self.check_object_spawn(req.marker_id_list[i]):
                aruco_np_pose = self.convert_pose_to_world(req.poses[i].position.x, req.poses[i].position.y, req.poses[i].position.z)
                self.move_model(req.marker_id_list[i], aruco_np_pose)
            else:
                aruco_np_pose = self.convert_pose_to_world(req.poses[i].position.x, req.poses[i].position.y, req.poses[i].position.z)
                self.spawn_model(req.marker_id_list[i], aruco_np_pose)
                self.spawned_object.append(req.marker_id_list[i])
                        
        resp.response = True

        return resp

    def euler_to_matrix(self, roll, pitch, yaw):
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])
        
        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])
        
        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])
    
        return R_z @ R_y @ R_x
    
    def convert_pose_to_world(self, pos_x, pos_y, pos_z):

        old_pos = np.array([pos_x, pos_y, pos_z])
        roll, pitch, yaw =  self.robot_rot_eulers[0], self.robot_rot_eulers[1], self.robot_rot_eulers[2]
        translation = np.array([self.robot_pose.position.x, self.robot_pose.position.y, self.robot_pose.position.z])

        rotation_matrix = self.euler_to_matrix(roll, pitch, yaw)

        new_pos = rotation_matrix @ old_pos + translation

        return new_pos

    def check_object_spawn(self, object_name):
        for i in self.spawned_object:
            if i == object_name:
                return True
        
        return False
    
    def move_model(self, object_name, np_pose):
        self.get_logger().info("Moving Model at : " + str(np_pose))
        
        move_object = MoveSim.Request()

        move_object.object_pose.position.x = np_pose[0]
        move_object.object_pose.position.y = np_pose[1]
        move_object.object_pose.position.z = np_pose[2]
        move_object.object_pose.orientation.x = 0.0
        move_object.object_pose.orientation.y = 0.0
        move_object.object_pose.orientation.z = 0.0
        move_object.object_pose.orientation.w = 1.0

        move_object.object_name = str(object_name)

        self.move_client.call_async(move_object)

    def spawn_model(self, object_name, np_pose):
        self.get_logger().info("Spawning Model at : " + str(np_pose))

        spawn_object = SpawnSim.Request()

        spawn_object.object_pose.position.x = np_pose[0]
        spawn_object.object_pose.position.y = np_pose[1]
        spawn_object.object_pose.position.z = np_pose[2]
        spawn_object.object_pose.orientation.x = 0.0
        spawn_object.object_pose.orientation.y = 0.0
        spawn_object.object_pose.orientation.z = 0.0
        spawn_object.object_pose.orientation.w = 1.0

        spawn_object.object_name = str(object_name)

        self.spawn_client.call_async(spawn_object)

def main():
    rclpy.init(args=None)
    node = SpawnIgn()
    while rclpy.ok():
        rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()




