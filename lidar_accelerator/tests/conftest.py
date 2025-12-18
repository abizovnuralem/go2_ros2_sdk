import sys
from pathlib import Path


_HERE = Path(__file__).resolve()

for p in _HERE.parents:
    pkg = p / "go2_robot_sdk"
    if pkg.is_dir():
        sys.path.insert(0, str(pkg))
        break
