#Just a test file, not really important fro now.

import json
import os
from pathlib import Path

from path_planner.obstacle_handler import ObstacleHandler

file_path = Path(__file__)

obs_original = os.path.join(str(file_path.parent.parent), 'data', 'obstacles.json')
with open(obs_original) as f:
    distros_dict = json.load(f)

for distro in distros_dict:
    print(distro())