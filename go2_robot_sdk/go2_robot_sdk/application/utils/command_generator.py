# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
Command generation utilities for Go2 robot.
Contains functions to create properly formatted WebRTC commands.
"""

import datetime
import json
import random
from typing import Any, Dict, Optional, Union

# Topic constants for different command types
SPORT_MODE_TOPIC = "rt/api/sport/request"
OBSTACLE_AVOIDANCE_TOPIC = "rt/api/obstacles_avoid/request"


def generate_id() -> int:
    """Generate a unique command ID based on timestamp and random number"""
    timestamp_part = int(datetime.datetime.now().timestamp() * 1000 % 2147483648)
    random_part = random.randint(0, 999)
    return timestamp_part + random_part


def create_command_structure(
        api_id: int, 
        parameter: Union[str, Dict], 
        topic: str = SPORT_MODE_TOPIC,
        command_id: Optional[int] = None
) -> Dict:
    """
    Create a standardized command structure for WebRTC communication.
    
    Args:
        api_id: API command identifier
        parameter: Command parameters (string or dict)
        topic: WebRTC topic for the command
        command_id: Optional specific command ID
        
    Returns:
        Dictionary containing the formatted command structure
    """
    final_id = generate_id() if command_id is None or command_id == 0 else command_id
    
    # Convert parameter to JSON string if it's a dict
    param_str = json.dumps(parameter) if isinstance(parameter, dict) else str(parameter)

    return {
        "type": "msg",
        "topic": topic,
        "data": {
            "header": {
                "identity": {
                    "id": final_id,
                    "api_id": api_id
                }
            },
            "parameter": param_str
        }
    }


def gen_command(
        cmd: int,
        parameters: Optional[Union[str, Dict]] = None,
        topic: Optional[str] = None,
        command_id: Optional[int] = None
) -> str:
    """
    Generate a general robot command.
    
    Args:
        cmd: Command ID from ROBOT_CMD constants
        parameters: Optional command parameters
        topic: Optional topic override
        command_id: Optional specific command ID
        
    Returns:
        JSON string of the formatted command
    """
    parameter = parameters if parameters is not None else str(cmd)
    command = create_command_structure(
        api_id=cmd,
        parameter=parameter,
        topic=topic or SPORT_MODE_TOPIC,
        command_id=command_id,
    )
    return json.dumps(command)


def gen_mov_command(
        x: float,
        y: float, 
        z: float,
        obstacle_avoidance: bool = False
) -> str:
    """
    Generate a movement command for the robot.
    
    Args:
        x: Forward/backward velocity
        y: Left/right velocity  
        z: Rotation velocity (yaw for obstacle avoidance)
        obstacle_avoidance: Whether to use obstacle avoidance mode
        
    Returns:
        JSON string of the formatted movement command
    """
    if obstacle_avoidance:
        # Obstacle avoidance uses different parameter format
        parameters = {"x": x, "y": y, "yaw": z, "mode": 0}
        command = create_command_structure(
            api_id=1003,  # Obstacle avoidance move command
            parameter=parameters,
            topic=OBSTACLE_AVOIDANCE_TOPIC,
        )
    else:
        # Standard sport mode movement
        parameters = {"x": x, "y": y, "z": z}
        command = create_command_structure(
            api_id=1008,  # Sport mode move command
            parameter=parameters,
            topic=SPORT_MODE_TOPIC,
        )

    return json.dumps(command) 