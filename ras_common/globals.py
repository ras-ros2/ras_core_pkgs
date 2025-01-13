import os

RAS_APP_NAME : str = None
RAS_APP_PATH : str = None
RAS_CONFIGS_PATH : str = None
RAS_WORKSPACE_PATH : str = None
RAS_ROBOT_MODE : str = None

def set_globals():
    global RAS_APP_NAME, RAS_APP_PATH, RAS_WORKSPACE_PATH, RAS_ROBOT_MODE, RAS_CONFIGS_PATH
    RAS_APP_NAME = os.environ.get('RAS_APP_NAME')
    RAS_APP_PATH = os.environ.get('RAS_APP_PATH')
    RAS_CONFIGS_PATH = os.path.join(RAS_APP_PATH, 'configs')
    RAS_WORKSPACE_PATH = os.environ.get('RAS_WORKSPACE_PATH')
    RAS_ROBOT_MODE = os.environ.get('RAS_ROBOT_MODE', 'sim')

set_globals()