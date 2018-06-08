import os
import sys

""" Set the path based on the operating system"""

if sys.platform.startswith('win'):
    config_dir = os.path.expanduser(os.path.join("~", ".abr_control"))
    cache_dir = os.path.join(config_dir, "cache")
    database_dir = os.path.joint(cache_dir, "abr_control_db.h5")
else:
    cache_dir = os.path.expanduser(os.path.join("~", ".cache", "abr_control"))
    database_dir = os.path.expanduser(os.path.join("~", ".cache",
        "abr_control", "abr_control_db.h5"))
