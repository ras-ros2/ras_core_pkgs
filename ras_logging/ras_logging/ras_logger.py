import os
import json
import yaml
import logging
import requests
from datetime import datetime
from pathlib import Path
from typing import Optional, Any

# Import RAS_APP_PATH or set a default
try:
    from ras_common.globals import RAS_APP_PATH
except ImportError:
    # Fallback if ras_common not installed
    RAS_APP_PATH = os.environ.get("RAS_APP_PATH", "/ras_robot_app")

# Try to import MQTT publisher if available
try:
    from ras_transport.interfaces.TransportWrapper import TransportMQTTPublisher
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False

# Define config paths following the same pattern as ras_common
CONFIG_PATH = Path(RAS_APP_PATH)/"configs"
CONFIG_FILE = CONFIG_PATH/"ras_conf.yaml"

class RasLogger:
    """
    Generic logger for RAS core packages. Logs info, error, JSON, and images to local files and a remote server.
    Remote server address is read from ras_conf.yaml.
    """
    def __init__(self, config_path: str = None):
        # Setup log directories
        self.base_log_dir = Path('logs')
        self.base_log_dir.mkdir(exist_ok=True)
        (self.base_log_dir / 'json').mkdir(exist_ok=True)
        (self.base_log_dir / 'images').mkdir(exist_ok=True)
        self.info_log = self.base_log_dir / 'info.log'
        self.error_log = self.base_log_dir / 'error.log'
        
        # Setup basic logger first
        self._setup_local_logger()
        
        # Use specified config path or use CONFIG_FILE from globals
        self.config_path = Path(config_path) if config_path else CONFIG_FILE
        
        if self.config_path.exists():
            self.logger.info(f"Using config file from: {self.config_path}")
        else:
            self.logger.warning(f"Config file not found at: {self.config_path}")
        
        self.remote_url = self._get_remote_url()
        
        # Setup MQTT publisher if available
        self.mqtt_publisher = None
        if MQTT_AVAILABLE:
            try:
                self.mqtt_publisher = TransportMQTTPublisher("last/will/topic")
                self.mqtt_publisher.connect_with_retries()
                self.logger.info("MQTT logging initialized")
            except Exception as e:
                self.logger.warning(f"Failed to initialize MQTT logging: {e}")
                self.mqtt_publisher = None

    def _setup_local_logger(self):
        self.logger = logging.getLogger('ras_logger')
        self.logger.setLevel(logging.INFO)
        # Info handler
        info_handler = logging.FileHandler(self.info_log)
        info_handler.setLevel(logging.INFO)
        info_handler.setFormatter(logging.Formatter('%(asctime)s [INFO] %(message)s'))
        # Error handler
        error_handler = logging.FileHandler(self.error_log)
        error_handler.setLevel(logging.ERROR)
        error_handler.setFormatter(logging.Formatter('%(asctime)s [ERROR] %(message)s'))
        # Add handlers
        self.logger.handlers = []
        self.logger.addHandler(info_handler)
        self.logger.addHandler(error_handler)

    def _get_remote_url(self) -> Optional[str]:
        # Reads config and returns remote log server URL
        try:
            with open(self.config_path, 'r') as f:
                conf = yaml.safe_load(f)
            
            # Get transport configuration, handling both schemas
            if 'ras' in conf and 'transport' in conf['ras']:
                transport = conf['ras']['transport']
                file_server = transport['file_server']
                
                # Log the actual values being used
                ip = file_server.get('ip')
                port = file_server.get('port')
                
                if not ip or not port:
                    self.logger.warning(f"Missing IP or port in config file. IP: {ip}, Port: {port}")
                    return None
                
                self.logger.info(f"Configuring log server with: {ip}:{port}")
                # Return base URL without /log path
                return f"http://{ip}:{port}"
            else:
                # Config file found but missing expected structure
                self.logger.warning(f"Config file structure invalid: 'ras' or 'transport' section missing")
                return None
        except Exception as e:
            self.logger.warning(f"Could not read remote log server from config: {e}")
            return None

    def log_info(self, message: str):
        """Log an info message."""
        self.logger.info(message)
        self._send_remote({'type': 'info', 'message': message, 'timestamp': self._now()})

    def log_error(self, message: str, exc: Exception = None):
        """Log an error message and optional exception."""
        full_msg = message
        if exc:
            full_msg += f"\nException: {repr(exc)}"
        self.logger.error(full_msg)
        self._send_remote({'type': 'error', 'message': full_msg, 'timestamp': self._now()})

    def log_json(self, data: Any, description: str = ""):
        """Log JSON data to a file and remote."""
        ts = self._now()
        fname = self.base_log_dir / 'json' / f'log_{ts}.json'
        with open(fname, 'w') as f:
            json.dump(data, f, indent=2)
        log_entry = {'type': 'json', 'description': description, 'timestamp': ts, 'data': data}
        self.logger.info(f"JSON logged: {fname} {description}")
        self._send_remote(log_entry)

    def log_image(self, image_bytes: bytes, description: str = "", ext: str = "jpg"):
        """Log image bytes to file and remote (as base64 or path)."""
        import base64
        ts = self._now()
        
        # Format description for filename (remove spaces and special chars)
        desc_for_filename = ""
        if description:
            desc_for_filename = "_" + description.replace(' ', '_').replace('/', '_')
            
        # Create the filename with timestamp and description
        img_filename = f"img_{ts}{desc_for_filename}.{ext}"
        img_path = self.base_log_dir / 'images' / img_filename
        
        # Save the image
        with open(img_path, 'wb') as f:
            f.write(image_bytes)
            
        # Get the absolute path for clickable links
        abs_path = os.path.abspath(img_path)
        
        # Format paths in various ways for different editor environments
        # Standard file:// URI format (works in many editors)
        file_uri = f"file://{abs_path}"
        
        # Convert any backslashes to forward slashes (for Windows compatibility)
        abs_path_fwd = abs_path.replace('\\', '/')
        
        # Log formats that are more likely to be clickable
        self.logger.info(f"Image logged: {img_path} {description}")
        
        # Add explicit format markers that VSCode recognizes
        self.logger.info(f"[IMAGE] {abs_path_fwd}")
        self.logger.info(f"file://{abs_path_fwd}")
        self.logger.info(f"Image can be found at {abs_path_fwd}")
        
        # For remote: send path and base64 (small images only, else path)
        try:
            img_b64 = base64.b64encode(image_bytes).decode('utf-8')
            payload = {'type': 'image', 'description': description, 'timestamp': ts, 'path': str(img_path), 'image_base64': img_b64}
            self._send_remote(payload)
        except Exception as e:
            self.logger.warning(f"Failed to send image to remote: {e}")

    def _send_remote(self, payload: dict):
        # Try to send via MQTT first if available
        if self.mqtt_publisher:
            try:
                self.mqtt_publisher.publish(payload)
                return  # If MQTT succeeds, no need to try HTTP
            except Exception as e:
                self.logger.warning(f"MQTT log failed: {e}")
        
        # Fall back to HTTP if MQTT fails or isn't available
        if not self.remote_url:
            return
        try:
            # Use /upload endpoint for file upload
            upload_url = f"{self.remote_url}/upload"
            
            # Create a unique filename based on timestamp and log type
            timestamp = self._now()
            log_type = payload.get('type', 'general')
            filename = f"log_{log_type}_{timestamp}.json"
            
            # Convert payload to JSON string
            payload_json = json.dumps(payload)
            
            # Create a temp file with the JSON data
            temp_file_path = self.base_log_dir / f"temp_{timestamp}.json"
            with open(temp_file_path, 'w') as f:
                f.write(payload_json)
            
            # Send as multipart/form-data with the file
            files = {'file': (filename, open(temp_file_path, 'rb'), 'application/json')}
            
            self.logger.info(f"Sending log to {upload_url}")
            response = requests.post(upload_url, files=files, timeout=2)
            
            # Clean up temp file
            temp_file_path.unlink(missing_ok=True)
            
            if response.status_code != 200:
                self.logger.warning(f"Remote log failed with status code: {response.status_code}")
        except Exception as e:
            self.logger.warning(f"Remote log failed: {e}")

    def _now(self):
        return datetime.now().strftime('%Y%m%d_%H%M%S')
        
    def __del__(self):
        # Clean up MQTT publisher if it exists
        if self.mqtt_publisher:
            try:
                self.mqtt_publisher.disconnect()
            except:
                pass

# Example usage:
# logger = RasLogger()
# logger.log_info("Robot started")
# logger.log_json({"state": "idle"}, "Current robot state")
# logger.log_image(img_bytes, "Camera snapshot")
# try:
#     risky_operation()
# except Exception as e:
#     logger.log_error("Operation failed", e)
