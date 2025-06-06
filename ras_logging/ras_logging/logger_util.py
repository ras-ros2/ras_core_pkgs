import logging
import sys
from logging.handlers import RotatingFileHandler
import json
from datetime import datetime
from pathlib import Path
class Colors:
    RESET = "\033[0m"
    INFO = "\033[38;5;110m"
    WARNING = "\033[38;5;226m"
    ERROR = "\033[38;5;174m"
    DEBUG = "\033[38;5;240m"
    CRITICAL = "\033[38;5;162m"
class CustomFormatter(logging.Formatter):
    def __init__(self, fmt, datefmt, use_colors=True):
        super().__init__(fmt, datefmt)
        self.use_colors = use_colors
    def format(self, record):
        # Pretty-print dicts/lists
        if isinstance(record.msg, (dict, list)):
            record.msg = json.dumps(record.msg, indent=2)
        # Try to trim the path to start from the main package directory
        path_str = str(record.pathname).replace('\\', '/')  # Normalize for Windows/Linux
        # Add more package roots as needed
        pkg_roots = [
            'ras_perception/',
            'ras_transport/',
            'ras_logging/',
            'ras_core_pkgs/',
            'ras_common/',
            'ras_bt_framework/',
            'ras_moveit/',
            'ras_interfaces/',
            'ras_sim/'
        ]
        for pkg in pkg_roots:
            # Try to find the last occurrence of the package in the path
            # (to avoid nested/duplicate package paths)
            lib_idx = path_str.rfind(f'/lib/{pkg}')
            if lib_idx != -1:
                trimmed_path = path_str[lib_idx + 5:]
                break
            install_idx = path_str.rfind(f'/install/{pkg}lib/{pkg}')
            if install_idx != -1:
                trimmed_path = path_str[install_idx + 9:]
                break
            pkg_idx = path_str.rfind(pkg)
            if pkg_idx != -1:
                trimmed_path = path_str[pkg_idx:]
                break
        else:
            # Fallback to filename only if no package found
            trimmed_path = Path(record.pathname).name
        record.file_info = f"{trimmed_path}:{record.lineno}"
        # Only colorize for console, never for file
        if self.use_colors:
            color_code = getattr(Colors, record.levelname, Colors.RESET)
            record.levelname = f"{color_code}{record.levelname}{Colors.RESET}"
            # Only color the message if not already colored
            if not (str(record.msg).startswith(color_code) and str(record.msg).endswith(Colors.RESET)):
                record.msg = f"{color_code}{record.msg}{Colors.RESET}"
        else:
            # Remove any color codes from levelname and msg for file output
            import re
            ansi_escape = re.compile(r'\x1b\[[0-9;]*m')
            record.levelname = ansi_escape.sub('', str(record.levelname))
            record.msg = ansi_escape.sub('', str(record.msg))
        return super().format(record)
def setup_logger(name: str = "app_logger"):
    Path("logs").mkdir(exist_ok=True)
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)
    if logger.handlers:
        return logger
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.DEBUG)
    file_handler = RotatingFileHandler(
        'logs/app.log',
        maxBytes=10*1024*1024,
        backupCount=5,
        encoding='utf-8'
    )
    file_handler.setLevel(logging.DEBUG)
    log_format = '[%(asctime)s] %(levelname)s [%(file_info)s] - %(message)s'
    console_formatter = CustomFormatter(log_format, datefmt='%Y-%m-%d %H:%M:%S', use_colors=True)
    file_formatter = CustomFormatter(log_format, datefmt='%Y-%m-%d %H:%M:%S', use_colors=False)
    console_handler.setFormatter(console_formatter)
    file_handler.setFormatter(file_formatter)
    logger.addHandler(console_handler)
    logger.addHandler(file_handler)
    return logger

logger = setup_logger()

