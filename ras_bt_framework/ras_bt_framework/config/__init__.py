"""
Configuration package for the behavior tree framework.
"""

from .action_mappings import action_mapping
from .default_actions import register_default_actions

__all__ = ['action_mapping', 'register_default_actions'] 