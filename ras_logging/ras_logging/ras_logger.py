import os
import json
import yaml
import logging
import requests
from datetime import datetime
from pathlib import Path
from typing import Optional, Any

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
        self.config_path = config_path or os.path.join(os.path.dirname(__file__), '../../../configs/ras_conf.yaml')
        self._setup_local_logger()
        self.remote_url = self._get_remote_url()

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
        # Reads ras_conf.yaml and returns remote log server URL
        try:
            with open(self.config_path, 'r') as f:
                conf = yaml.safe_load(f)
            file_server = conf['ras']['transport']['file_server']
            ip = file_server['ip']
            port = file_server['port']
            # Example: http://localhost:2122/log
            return f"http://{ip}:{port}/log"
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
        fname = self.base_log_dir / 'images' / f'img_{ts}.{ext}'
        with open(fname, 'wb') as f:
            f.write(image_bytes)
        self.logger.info(f"Image logged: {fname} {description}")
        # For remote: send path and base64 (small images only, else path)
        try:
            img_b64 = base64.b64encode(image_bytes).decode('utf-8')
            payload = {'type': 'image', 'description': description, 'timestamp': ts, 'path': str(fname), 'image_base64': img_b64}
            self._send_remote(payload)
        except Exception as e:
            self.logger.warning(f"Failed to send image to remote: {e}")

    def _send_remote(self, payload: dict):
        if not self.remote_url:
            return
        try:
            requests.post(self.remote_url, json=payload, timeout=2)
        except Exception as e:
            self.logger.warning(f"Remote log failed: {e}")

    def _now(self):
        return datetime.now().strftime('%Y%m%d_%H%M%S')

# Example usage:
# logger = RasLogger()
# logger.log_info("Robot started")
# logger.log_json({"state": "idle"}, "Current robot state")
# logger.log_image(img_bytes, "Camera snapshot")
# try:
#     risky_operation()
# except Exception as e:
#     logger.log_error("Operation failed", e)
