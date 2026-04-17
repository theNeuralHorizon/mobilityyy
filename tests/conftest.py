import sys
from pathlib import Path

# Add ROS 2 package source directories to sys.path so pytest can import them
# without requiring a full colcon build + install.
_src = Path(__file__).resolve().parent.parent / "src"
for pkg_dir in ("rover_autonomy", "rover_logging"):
    pkg_path = str(_src / pkg_dir)
    if pkg_path not in sys.path:
        sys.path.insert(0, pkg_path)
