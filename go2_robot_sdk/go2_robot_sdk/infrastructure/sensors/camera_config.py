# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
Camera configuration loader for Go2 robot.
Loads camera calibration data from YAML files for different resolutions.
"""

import yaml
import logging
import glob
import os
import re
from typing import Dict, Optional
from sensor_msgs.msg import CameraInfo
from ament_index_python.packages import get_package_share_directory

logger = logging.getLogger(__name__)


class CameraConfigLoader:
    """Loader for camera calibration configurations"""
    
    def __init__(self, package_name: str = 'go2_robot_sdk'):
        self.package_name = package_name
        self._camera_info_cache: Optional[Dict[int, CameraInfo]] = None
    
    def get_supported_resolutions(self) -> list[int]:
        """Get list of supported camera resolutions"""
        try:
            calibration_dir = os.path.join(
                get_package_share_directory(self.package_name),
                "calibration"
            )
            
            pattern = os.path.join(calibration_dir, "front_camera_*.yaml")
            files = glob.glob(pattern)
            
            resolutions = []
            for file_path in files:
                filename = os.path.basename(file_path)
                numbers = re.findall(r"\d+", filename)
                if numbers:
                    resolutions.append(int(numbers[0]))
            
            return sorted(resolutions)
            
        except Exception as e:
            logger.error(f"Failed to get supported resolutions: {e}")
            return []
    
    def load_camera_info_for_resolution(self, height: int) -> Optional[CameraInfo]:
        """
        Load camera info for specific resolution.
        
        Args:
            height: Image height (resolution identifier)
            
        Returns:
            CameraInfo message or None if loading fails
        """
        try:
            yaml_file = os.path.join(
                get_package_share_directory(self.package_name),
                "calibration",
                f"front_camera_{height}.yaml"
            )
            
            if not os.path.exists(yaml_file):
                logger.warning(f"Camera calibration file not found: {yaml_file}")
                return None
            
            logger.info(f"Loading camera info from file: {yaml_file}")
            
            with open(yaml_file, "r") as file_handle:
                camera_data = yaml.safe_load(file_handle)
            
            # Create and populate CameraInfo message
            camera_info = CameraInfo()
            camera_info.width = camera_data["image_width"]
            camera_info.height = camera_data["image_height"]
            camera_info.k = camera_data["camera_matrix"]["data"]
            camera_info.d = camera_data["distortion_coefficients"]["data"]
            camera_info.r = camera_data["rectification_matrix"]["data"]
            camera_info.p = camera_data["projection_matrix"]["data"]
            camera_info.distortion_model = camera_data["distortion_model"]
            
            return camera_info
            
        except Exception as e:
            logger.error(f"Failed to load camera info for height {height}: {e}")
            return None
    
    def load_all_camera_info(self) -> Dict[int, CameraInfo]:
        """
        Load camera info for all supported resolutions.
        
        Returns:
            Dictionary mapping resolution height to CameraInfo messages
        """
        if self._camera_info_cache is not None:
            return self._camera_info_cache
        
        supported_heights = self.get_supported_resolutions()
        logger.info(f"Loading camera info for heights: {supported_heights}")
        
        camera_info_dict = {}
        
        for height in supported_heights:
            camera_info = self.load_camera_info_for_resolution(height)
            if camera_info is not None:
                camera_info_dict[height] = camera_info
        
        self._camera_info_cache = camera_info_dict
        return camera_info_dict
    
    def get_camera_info(self, height: int) -> Optional[CameraInfo]:
        """
        Get camera info for specific height with caching.
        
        Args:
            height: Image height
            
        Returns:
            CameraInfo message or None if not available
        """
        if self._camera_info_cache is None:
            self.load_all_camera_info()
        
        return self._camera_info_cache.get(height)


# Global loader instance for backward compatibility
_camera_loader: Optional[CameraConfigLoader] = None


def get_camera_loader() -> CameraConfigLoader:
    """Get singleton camera config loader instance"""
    global _camera_loader
    if _camera_loader is None:
        _camera_loader = CameraConfigLoader()
    return _camera_loader


def load_camera_info() -> Dict[int, CameraInfo]:
    """
    Load camera info for all supported resolutions.
    Backward compatibility function.
    
    Returns:
        Dictionary mapping resolution height to CameraInfo messages
    """
    loader = get_camera_loader()
    return loader.load_all_camera_info() 