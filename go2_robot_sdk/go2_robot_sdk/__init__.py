import sys
import os
import logging
from ament_index_python import get_package_share_directory

logger = logging.getLogger(__name__)

libs_path =  os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'external_lib'
    )
sys.path.insert(0, os.path.join(libs_path, 'aioice'))
sys.path.insert(0, os.path.join(libs_path))

logger.info('Patched aioice added to sys.path: {}'.format(sys.path))