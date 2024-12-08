import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
import json
import time
from oss_interfaces.msg import Instruction
from std_msgs.msg import Bool


# AWS IoT Parameters
host = "a1u5bxro5znv23-ats.iot.ap-south-1.amazonaws.com"
root_ca_path = "/workspaces/robo_arm_ws/src/xarm_cpp_custom/AWS_Cert/AmazonRootCA1.pem"
certificate_path = "/workspaces/robo_arm_ws/src/xarm_cpp_custom/AWS_Cert/4f163fd4aedc5fb7dac84fb40685a69859a5eec8b47351d0cbc7e393e1101588-certificate.pem.crt"
private_key_path = "/workspaces/robo_arm_ws/src/xarm_cpp_custom/AWS_Cert/4f163fd4aedc5fb7dac84fb40685a69859a5eec8b47351d0cbc7e393e1101588-private.pem.key"
port = 8883
topic = "status/topic"
client_id = "receiverClientNew"

class AwsStatusReceiver(Node):
    def __init__(self):
        super().__init__('trajectory_receiver')

        self.get_logger().info("NODE STARTED")
        
        self.publisher_ = self.create_publisher(JointTrajectory, 'trajectory_topic', 10)
        self.publisher_instruction = self.create_publisher(Instruction, 'instruction_topic', 10)

        # Initialize the MQTT client
        self.mqtt_client = AWSIoTMQTTClient(client_id)
        self.mqtt_client.configureEndpoint(host, port)
        self.mqtt_client.configureCredentials(root_ca_path, private_key_path, certificate_path)
        self.mqtt_client.configureOfflinePublishQueueing(-1)  # Infinite offline publish queueing
        self.mqtt_client.configureDrainingFrequency(2)  # Draining: 2 Hz
        self.mqtt_client.configureConnectDisconnectTimeout(10.0)  # 10 sec
        self.mqtt_client.configureMQTTOperationTimeout(10.0)  # 10 sec
        self.mqtt_client.configureLastWill("last/will/topic", "Client disconnected unexpectedly", 1)
        self.instruction_msg = []

        self.instruction_flag = True

        self.pub_aws_logger = self.create_publisher(Bool, "/logger_status", 10)
        self.pub_aws_receiver = self.create_publisher(Bool, "/aws_sender_status", 10)
        self.pub_arm_status = self.create_publisher(Bool, "/arm_status_web", 10)


        # Connect to AWS IoT
        self.connect_to_aws()

        # Subscribe to the topic
        self.mqtt_client.subscribe(topic, 1, self.custom_callback)
        self.get_logger().info(f"Subscribed to topic: {topic}")
        self.payload = ''

    def connect_to_aws(self):
        """Attempt to connect to AWS IoT with retries"""
        while True:
            try:
                self.mqtt_client.connect()
                self.get_logger().info("Connected to AWS IoT")
                break
            except Exception as e:
                self.get_logger().error(f"Connection to AWS IoT failed: {e}. Retrying in 5 seconds...")
                time.sleep(5)

    def custom_callback(self, client, userdata, message):
        self.payload =  message.payload.decode("utf-8")
        self.get_logger().info("Received Message")

        if not self.payload:
            self.get_logger().info("Received an empty payload.")
            return

        try:
            log_data = json.loads(self.payload)
            receiver_data = Bool()
            logger_data = Bool()
            arm_data = Bool()

            receiver_data.data = bool(log_data["receiver"])
            logger_data.data = bool(log_data["logger"])
            arm_data.data = bool(log_data["arm"])

            self.pub_aws_receiver.publish(receiver_data)
            self.pub_aws_logger.publish(logger_data)
            self.pub_arm_status.publish(arm_data)
            self.get_logger().info(f"{log_data}")

        except json.JSONDecodeError as e:
            pass
            # self.get_logger().error(f"JSONDecodeError: {e}")
        except KeyError as e:
            pass
            # self.get_logger().error(f"KeyError: {e}")
        except Exception as e:
            # pass
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    receiver = AwsStatusReceiver()
    try:
        rclpy.spin(receiver)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup and disconnect
        receiver.destroy_node()
        receiver.mqtt_client.disconnect()
        receiver.get_logger().info("Disconnected from AWS IoT")
        rclpy.shutdown()

if __name__ == '__main__':
    main()
