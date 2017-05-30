import os
import sys


if sys.platform.startswith('win'):
    cache_dir = os.path.join(config_dir, "cache")
else:
    cache_dir = os.path.expanduser(os.path.join("~", ".cache", "abr_control"))
