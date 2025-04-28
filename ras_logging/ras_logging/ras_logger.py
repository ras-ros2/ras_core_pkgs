import os
import json
from datetime import datetime
from pathlib import Path
from typing import Optional, Any
import traceback
from .logger_util import logger

try:
    from ras_common.globals import RAS_APP_PATH
except ImportError:
    RAS_APP_PATH = os.environ.get("RAS_APP_PATH", "/ras_robot_app")

try:
    from ras_transport.interfaces.TransportWrapper import TransportMQTTPublisher
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False

CONFIG_PATH = Path(RAS_APP_PATH)/"configs"
CONFIG_FILE = CONFIG_PATH/"ras_conf.yaml"

class RasLogger:
    """
    Logger for RAS core packages. Logs info, error, JSON, and images to local files only using a shared logger. No network, MQTT, or remote logic.
    """
    def __init__(self, config_path: str = None):
        self.base_log_dir = Path('logs')
        self.base_log_dir.mkdir(exist_ok=True)
        (self.base_log_dir / 'json').mkdir(exist_ok=True)
        (self.base_log_dir / 'images').mkdir(exist_ok=True)

    def log_info(self, message: str):
        """Log an info message."""
        logger.info(message, stacklevel=2)

    def log_error(self, message: str, exc: Exception = None):
        """Log an error message and optional exception."""
        full_msg = message
        if exc:
            full_msg += f"\nException: {repr(exc)}"
        logger.error(full_msg, stacklevel=2)

    def log_json(self, data: Any, description: str = ""):
        """Log JSON data to a local file only."""
        ts = self._now()
        fname = self.base_log_dir / 'json' / f'log_{ts}.json'
        with open(fname, 'w') as f:
            json.dump(data, f)
        logger.info(f"JSON logged: {fname} {description}", stacklevel=2)

    def log_image(self, image_bytes: bytes, description: str = "", ext: str = "jpg"):
        """Log image bytes to file."""
        import base64
        ts = self._now()
        
        desc_for_filename = ""
        if description:
            desc_for_filename = "_" + description.replace(' ', '_').replace('/', '_')
        img_filename = f"img_{ts}{desc_for_filename}.{ext}"
        img_path = self.base_log_dir / 'images' / img_filename
        with open(img_path, 'wb') as f:
            f.write(image_bytes)
        abs_path = os.path.abspath(img_path)
        abs_path_fwd = abs_path.replace('\\', '/')
        logger.info(f"Image logged: {img_path} {description}", stacklevel=2)
        logger.info(f"[IMAGE] {abs_path_fwd}", stacklevel=2)
        logger.info(f"file://{abs_path_fwd}", stacklevel=2)
        logger.info(f"Image can be found at {abs_path_fwd}", stacklevel=2)

    def log_warn(self, message: str):
        logger.warning(message, stacklevel=2)

    def log_debug(self, message: str):
        logger.debug(message, stacklevel=2)
        
    def _now(self):
        return datetime.now().strftime('%Y%m%d_%H%M%S')