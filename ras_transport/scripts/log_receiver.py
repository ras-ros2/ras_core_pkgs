#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ras_interfaces.srv import JointSat, LoadExp, ArucoPoses, GripperLog
from std_srvs.srv import SetBool
from builtin_interfaces.msg import Duration
# from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
from rclpy.callback_groups import ReentrantCallbackGroup
from ras_transport.interfaces.TransportWrapper import TransportMQTTSubscriber
from ras_transport.interfaces.TransportWrapper import TransportFileClient
from ras_common.package.utils import get_cmake_python_pkg_source_dir
from ras_common.globals import RAS_APP_PATH, RAS_CONFIGS_PATH
from geometry_msgs.msg import Pose, Point, Quaternion
import os
import json
import yaml
from pathlib import Path
import time
import datetime
import re
from ras_logging.ras_logger import RasLogger
from std_msgs.msg import Bool

class TrajectoryLogger(LifecycleNode):
    def __init__(self):
        super().__init__('trajectory_logger')
        self.logger = RasLogger()

        my_callback_group = ReentrantCallbackGroup()
        self.publisher_ = self.create_publisher(JointTrajectory, 'trajectory_topic', 10)
        self.service_sync = self.create_client(JointSat, "sync_arm", callback_group=my_callback_group)
        self.fallback_client = self.create_client(LoadExp, "/fallback_info", callback_group=my_callback_group)
        # Initialize the MQTT client
        
        self.aruco_client = self.create_client(ArucoPoses, "/aruco_poses", callback_group=my_callback_group)

        self.instruction_msg = []
        self.file_client = TransportFileClient()
        
        # Set default experiment name and timestamp
        self.experiment_name = "0_stack"  # Default to 0_stack
        self.timestamp = self._get_timestamp_folder()
        
        # Ensure logs base path exists
        self.logs_base_path = Path(RAS_APP_PATH) / "logs"
        self.logs_base_path.mkdir(parents=True, exist_ok=True)
        
        # Create experiment-specific directory with timestamp
        self.experiment_log_path = self.logs_base_path / self.experiment_name / self.timestamp
        self.experiment_log_path.mkdir(parents=True, exist_ok=True)
        
        # Create subdirectories for different data types
        self.images_path = self.experiment_log_path / "image"
        self.images_path.mkdir(parents=True, exist_ok=True)
        
        self.json_path = self.experiment_log_path / "json"
        self.json_path.mkdir(parents=True, exist_ok=True)
        
        self.mqtt_sub = TransportMQTTSubscriber("last/will/topic", self.custom_callback)
        self.instruction_flag = True
        self.aruco_sync_flag = True

        # Initialize latest gripper message
        self.latest_gripper_msg = None
        
        # Subscribers
        self.subscription = self.create_subscription(
            Bool,  # The type of message
            'gripper_status',  # Topic name
            self.listener_callback,  # The callback function to call when a message is received
            10  # QoS (Quality of Service) depth
        )
        self.subscription

        # Connect to AWS IoT
        self.connect_to_aws()

        self.payload = ''

    def listener_callback(self, msg):
        self.latest_gripper_msg = msg
        if msg.data:
            self.get_logger().info('Gripper is ON')
        else:
            self.get_logger().info('Gripper is OFF')

    def gripper_callback(self, msg):
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S-%f")
        gripper_state = msg.data  # Assuming msg.data is the gripper state (True or False)
        save_path = os.path.join(self.gripper_save_dir, f"gripper_{timestamp}.txt")
        
        with open(save_path, 'w') as f:
            f.write(str(gripper_state))  # Save the gripper state as text (True/False)

        self.logger.log_info(f"Saved gripper state at {save_path}")

    def _get_current_experiment_name(self):
        """Get the name of the currently loaded experiment"""
        # First check for current_experiment.txt in RAS_APP_PATH (highest priority)
        exp_path = Path(RAS_APP_PATH) / "current_experiment.txt"
        if exp_path.exists():
            try:
                with open(exp_path, 'r') as f:
                    content = f.read().strip()
                    if content:
                        self.logger.log_info(f"Using experiment name from {exp_path}: {content}")
                        return content
            except Exception as e:
                self.logger.log_warn(f"Error reading {exp_path}: {e}")
        
        # If not found or error, try other locations
        try:
            # Try in the configs directory
            configs_exp_path = Path(RAS_CONFIGS_PATH) / "current_experiment.txt"
            if configs_exp_path.exists():
                with open(configs_exp_path, 'r') as f:
                    content = f.read().strip()
                    if content:
                        self.logger.log_info(f"Using experiment name from {configs_exp_path}: {content}")
                        return content
                        
            # Try the yaml file format
            yaml_path = Path(RAS_CONFIGS_PATH) / "current_experiment.yaml"
            if yaml_path.exists():
                with open(yaml_path, 'r') as f:
                    content = yaml.safe_load(f)
                    if content and 'experiment_id' in content:
                        exp_id = content['experiment_id']
                        self.logger.log_info(f"Using experiment ID from yaml config: {exp_id}")
                        return exp_id
        except Exception as e:
            self.logger.log_warn(f"Error reading experiment configuration: {e}")
        
        # If everything fails, return the default
        self.logger.log_info("Using default experiment name: 0_stack")
        return "0_stack"

    def _get_timestamp_folder(self):
        """Generate a timestamp for folder name"""
        return datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

    def connect_to_aws(self):
        self.mqtt_sub.connect_with_retries()
        
    def custom_callback(self, message):
        self.logger.log_info(f"message is {message} and type is {type(message)}")
        self.payload =  message.decode("utf-8")
        self.logger.log_info(f"now type is {type(self.payload)}")
        self.logger.log_info("Received Message")

        # Append only log lines to app.log
        server_log_path = Path(RAS_APP_PATH) / "logs" / "app.log"
        appended = False

        # If it's plain text (not JSON), treat as log line
        if not (self.payload.strip().startswith("{") or self.payload.strip().startswith("[")):
            tagged_line = f"[SLAVE] {self.payload.strip()}\n" if self.payload.strip() else self.payload
            self.logger.log_info(f"[DEBUG] Appending to server app.log: {tagged_line.strip()}")
            with open(server_log_path, "a") as f:
                f.write(tagged_line)
            appended = True

        if not self.payload:
            self.logger.log_info("Received an empty payload.")
            return

        try:
            # Get the current experiment name before processing
            current_experiment = self._get_current_experiment_name()
            if current_experiment != self.experiment_name:
                self.experiment_name = current_experiment
                self.logger.log_info(f"Updated experiment name to: {self.experiment_name}")
                
                # Update paths with new experiment name
                self.experiment_log_path = self.logs_base_path / self.experiment_name / self.timestamp
                self.experiment_log_path.mkdir(parents=True, exist_ok=True)
                
                # Update subdirectories for different data types
                self.images_path = self.experiment_log_path / "image"
                self.images_path.mkdir(parents=True, exist_ok=True)
                
                self.json_path = self.experiment_log_path / "json"
                self.json_path.mkdir(parents=True, exist_ok=True)
            
            self.logger.log_info(f"payload: {self.payload[:200]}...")  # Print first 200 chars to avoid flooding logs
            log_data = json.loads(self.payload)
            self.logger.log_info(f"Log data type: {log_data.get('type')}")

            # Append multiple log lines if 'logs' field is present
            logs_list = log_data.get("logs")
            if logs_list and isinstance(logs_list, list):
                for log_line in logs_list:
                    if log_line.strip():
                        tagged_line = f"[SLAVE] {log_line.strip()}\n"
                        self.logger.log_info(f"[DEBUG] Appending JSON log line to server app.log: {tagged_line.strip()}")
                        with open(server_log_path, "a") as f:
                            f.write(tagged_line)
                appended = True
            # Fallback: Append only if log_data contains a single log line
            log_line = log_data.get("log") or log_data.get("log_line")
            if log_line and not appended:
                tagged_line = f"[SLAVE] {log_line.strip()}\n"
                self.logger.log_info(f"[DEBUG] Appending JSON log line to server app.log: {tagged_line.strip()}")
                with open(server_log_path, "a") as f:
                    f.write(tagged_line)
                appended = True

            # Extract and print image filename for IoT receiver
            pkg_path = get_cmake_python_pkg_source_dir("ras_app_main")
            extract_path = str(self.images_path)
            Path(extract_path).mkdir(parents=True, exist_ok=True)
            image_filename = log_data.get("image_filename")
            if image_filename:
                self.logger.log_info(f"Received image filename from MQTT: {image_filename}")
                # Download the image file using the file client
                img_result = self.file_client.download(image_filename, f"{extract_path}/{image_filename}")
                if img_result:
                    self.logger.log_info(f"Successfully downloaded image: {extract_path}/{image_filename}")
                else:
                    self.logger.log_warn(f"Failed to download image: {image_filename}")
            else:
                self.logger.log_warn("No image filename found in this message.")

            # Handle image type specially
            if log_data.get('type') == 'image' and 'image_base64' in log_data:
                self.logger.log_info("Found image type with image_base64, handling image")
                self._handle_image_log(log_data)
            
            # Save JSON data with a consistent filename format regardless of type
            step_num = log_data.get('step', '').replace('step', '').strip()
            
            # Try to extract step number from image_filename if it exists
            if not step_num and image_filename:
                # Parse step number from strings like "img_20250423_182441_Color_image_for_step_1.jpg"
                if "for_step_" in image_filename:
                    try:
                        step_part = image_filename.split("for_step_")[1]
                        # Extract digits until non-digit character (typically .jpg)
                        import re
                        step_match = re.search(r'(\d+)', step_part)
                        if step_match:
                            step_num = step_match.group(1)
                            self.logger.log_info(f"Extracted step number '{step_num}' from image filename")
                    except Exception as e:
                        self.logger.log_warn(f"Error extracting step number from image filename: {e}")
            
            # Check other fields if still not found
            if not step_num and 'current_traj' in log_data:
                step_num = str(log_data.get('current_traj', ''))
                self.logger.log_info(f"Using current_traj value '{step_num}' as step number")
            if not step_num:
                step_num = "unknown"
            
            # Set step number in current_traj if necessary
            if 'current_traj' in log_data and not isinstance(log_data['current_traj'], int):
                try:
                    # Try to convert step_num to integer if it's not already
                    log_data['current_traj'] = int(step_num) if step_num.isdigit() else 0
                    self.logger.log_info(f"Updated current_traj to: {log_data['current_traj']}")
                except Exception as e:
                    self.logger.log_warn(f"Failed to update current_traj: {e}")
            
            # Update traj_status to include step number if it's currently just SUCCESS/FAILED
            if 'traj_status' in log_data:
                if log_data['traj_status'] in ['SUCCESS', 'FAILED']:
                    orig_status = log_data['traj_status']
                    log_data['traj_status'] = f"{orig_status}_{step_num}"
                    self.logger.log_info(f"Updated traj_status to include step: {log_data['traj_status']}")

            # Add gripper status to the log_data
            if self.latest_gripper_msg is not None:
                gripper_state = self.latest_gripper_msg.data
                log_data['gripper_status'] = str(gripper_state)
                self.logger.log_info(f"Added gripper status to log data: {gripper_state}")

            # Format timestamp similar to image filename format
            formatted_timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            
            # Get a meaningful description of the data type
            data_type = log_data.get('type', '')
            description = log_data.get('description', '')
            
            # Create a descriptive part similar to "Color_image" in the example
            if description:
                descriptive_part = f"{data_type}_{description}".replace(' ', '_')
            else:
                descriptive_part = data_type if data_type else "data"
            
            # Create JSON filename pattern similar to image pattern:
            # img_20250423_182441_Color_image_for_step_1.jpg -> json_20250423_182441_trajectory_data_for_step_1.json
            json_filename = f"json_{formatted_timestamp}_{descriptive_part}_for_step_{step_num}.json"
            json_file_path = self.json_path / json_filename
            
            # Remove logs field from the log data before saving
            if 'logs' in log_data:
                # self.logger.log_info("Removing 'logs' field from JSON data before saving")
                log_data_to_save = log_data.copy()
                log_data_to_save.pop('logs', None)
            else:
                log_data_to_save = log_data
            
            if image_filename:
                with open(json_file_path, 'w') as f:
                    json.dump(log_data_to_save, f, indent=2)
                self.logger.log_info(f"Saved JSON data to: {json_file_path}")

            if self.aruco_sync_flag == False:
                aruco_pose = ArucoPoses.Request()
                poses_list = json.loads(log_data["aruco_markers"])["poses"]
                marker_id_list = json.loads(log_data["aruco_markers"])["marker_ids"]

                aruco_pose.marker_id_list = marker_id_list

                aruco_pose.poses = [
                    Pose(
                        position=Point(x=pose["position"]["x"], y=pose["position"]["y"], z=pose["position"]["z"]),
                        orientation=Quaternion(x=pose["orientation"]["x"], y=pose["orientation"]["y"], z=pose["orientation"]["z"], w=pose["orientation"]["w"])
                    ) for pose in poses_list
                ]
                
                self.aruco_client.call_async(aruco_pose)
                self.logger.log_info("Spawing Started...")
                # self.aruco_sync_flag = False
                
            if log_data.get("traj_status") == "SUCCESS" or log_data.get("traj_status", "").startswith("SUCCESS_"):
                self.logger.log_info("SUCCESS")
                request = JointSat.Request()
                request.joint_state.position = log_data["joint_state"]
                self.future = self.service_sync.call_async(request)
                rclpy.spin_until_future_complete(self, self.future)

            if log_data.get("traj_status") == "FAILED" or log_data.get("traj_status", "").startswith("FAILED_"):
                self.logger.log_info("FAILED")
                # request = SetBool.Request()
                # request.data = True
                # self.future = self.service_.call_async(request)
                request = LoadExp.Request()
                # Use the step number from traj_status or current_traj
                if step_num and step_num.isdigit():
                    request.instruction_no = step_num
                else:
                    request.instruction_no = str(log_data.get("current_traj", 0))
                request.picked_object = "beaker-1"
                self.future2 = self.fallback_client.call_async(request)
                rclpy.spin_until_future_complete(self, self.future2)

        except json.JSONDecodeError as e:
            pass
            # self.logger.log_error(f"JSONDecodeError: {e}")
        except KeyError as e:
            pass
            # self.logger.log_error(f"KeyError: {e}")
        except Exception as e:
            self.logger.log_error(f"Error: {e}", e)
            
    def _handle_image_log(self, log_data):
        """Extract and save image data from log entries with type 'image'"""
        try:
            import base64
            import re
            
            # Use the experiment-specific image directory
            images_dir = self.images_path
            
            self.logger.log_info(f"Handling image data with keys: {list(log_data.keys())}")
            if 'image_base64' not in log_data:
                self.logger.log_warn(f"Warning: No image_base64 found in log_data, skipping image processing")
                return
                
            # Extract metadata
            timestamp = log_data.get('timestamp', self._get_timestamp())
            description = log_data.get('description', 'image')
            ext = 'jpg'  # Default extension
            
            # Extract step number if available from various sources
            step_num = log_data.get('step', '').replace('step', '').strip()
            
            # Try to extract from path if present
            if not step_num and 'path' in log_data:
                path = log_data['path']
                if "for_step_" in path:
                    try:
                        step_part = path.split("for_step_")[1]
                        step_match = re.search(r'(\d+)', step_part)
                        if step_match:
                            step_num = step_match.group(1)
                            self.logger.log_info(f"Extracted step number '{step_num}' from image path")
                    except Exception:
                        pass
                        
            # Try to extract from image_filename if present
            if not step_num and 'image_filename' in log_data:
                image_filename = log_data['image_filename']
                if "for_step_" in image_filename:
                    try:
                        step_part = image_filename.split("for_step_")[1]
                        step_match = re.search(r'(\d+)', step_part)
                        if step_match:
                            step_num = step_match.group(1)
                            self.logger.log_info(f"Extracted step number '{step_num}' from image_filename")
                    except Exception:
                        pass
                        
            # If still not found, check other fields
            if not step_num and 'current_traj' in log_data:
                step_num = log_data.get('current_traj', '')
            if not step_num:
                step_num = "unknown"
            
            # Determine file extension from path if available
            if 'path' in log_data:
                path = log_data['path']
                self.logger.log_info(f"Image path: {path}")
                if '.' in path:
                    ext = path.split('.')[-1]
            
            # Create a unique filename that includes the step number
            image_path = images_dir / f"img_{timestamp}_{description.replace(' ', '_')}_for_step_{step_num}.{ext}"
            self.logger.log_info(f"Will save image to: {image_path}")
            
            # Decode base64 image and save to file
            image_data = base64.b64decode(log_data['image_base64'])
            self.logger.log_info(f"Decoded image data length: {len(image_data)} bytes")
            with open(image_path, 'wb') as f:
                f.write(image_data)
                
            self.logger.log_info(f"Saved image to: {image_path}")
            
            # Add gripper status to metadata
            gripper_status = "unknown"
            if self.latest_gripper_msg is not None:
                gripper_status = str(self.latest_gripper_msg.data)
            
            # Create metadata for this image (to be saved as JSON)
            log_meta = {
                'type': 'image_metadata',
                'timestamp': timestamp,
                'description': description,
                'step': step_num,
                'saved_path': str(image_path),
                'gripper_status': gripper_status,
            }
            
            # Remove logs from original_image_data if present before adding to metadata
            original_data = log_data.copy()
            if 'logs' in original_data:
                self.logger.log_info("Removing 'logs' field from image metadata")
                original_data.pop('logs', None)
            log_meta['original_image_data'] = original_data
            
            # Create JSON filename pattern for image metadata using the same step number
            json_filename = f"json_{timestamp}_image_metadata_for_step_{step_num}.json"
            json_file_path = self.json_path / json_filename
            
            with open(json_file_path, 'w') as f:
                json.dump(log_meta, f, indent=2)
            self.logger.log_info(f"Saved image metadata to: {json_file_path}")
                
        except Exception as e:
            import traceback
            print(f"Error in _handle_image_log: {e}")
            print(traceback.format_exc())
            self.get_logger().error(f"Failed to save image: {e}")
    
    def _get_timestamp(self):
        """Generate a timestamp string"""
        return datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

def main(args=None):
    rclpy.init(args=args)
    receiver = TrajectoryLogger()
    try:
        while rclpy.ok():
            rclpy.spin_once(receiver,timeout_sec=0.1)
            receiver.mqtt_sub.loop()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup and disconnect
        receiver.destroy_node()
        receiver.mqtt_sub.disconnect()
        receiver.logger.log_info("Disconnected from AWS IoT")
        rclpy.shutdown()

if __name__ == '__main__':
    main()