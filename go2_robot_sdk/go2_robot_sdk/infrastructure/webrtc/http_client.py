# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
HTTP client for Go2 WebRTC signaling.
Handles HTTP communication with robot for establishing WebRTC connections.
"""

import logging
import requests
from typing import Optional, Dict, Any
from requests.exceptions import RequestException, HTTPError, ConnectionError, Timeout


logger = logging.getLogger(__name__)


class WebRTCHttpError(Exception):
    """Custom exception for WebRTC HTTP communication errors"""
    pass


class HttpClient:
    """HTTP client for WebRTC signaling communication"""
    
    def __init__(self, timeout: float = 10.0):
        """
        Initialize HTTP client.
        
        Args:
            timeout: Request timeout in seconds
        """
        self.timeout = timeout
        self.session = requests.Session()
        
        # Set default headers
        self.session.headers.update({
            'User-Agent': 'Go2WebRTC/1.0',
            'Accept': 'application/json, text/plain, */*',
            'Connection': 'keep-alive'
        })
    
    def make_request(
        self, 
        url: str, 
        method: str = 'POST',
        body: Optional[str] = None, 
        headers: Optional[Dict[str, str]] = None
    ) -> Optional[requests.Response]:
        """
        Make HTTP request with error handling.
        
        Args:
            url: Request URL
            method: HTTP method (GET, POST, etc.)
            body: Request body data
            headers: Additional headers
            
        Returns:
            Response object or None if request failed
            
        Raises:
            WebRTCHttpError: For various HTTP communication errors
        """
        try:
            # Merge additional headers with session headers
            request_headers = self.session.headers.copy()
            if headers:
                request_headers.update(headers)
            
            logger.debug(f"Making {method} request to {url}")
            logger.debug(f"Headers: {request_headers}")
            logger.debug(f"Body: {body}")
            
            # Make the request
            response = self.session.request(
                method=method,
                url=url,
                data=body,
                headers=request_headers,
                timeout=self.timeout
            )
            
            # Check if the request was successful
            response.raise_for_status()
            
            logger.debug(f"Response status: {response.status_code}")
            logger.debug(f"Response body: {response.text}")
            
            return response
            
        except ConnectionError as e:
            error_msg = f"Connection error when requesting {url}: {e}"
            logger.error(error_msg)
            raise WebRTCHttpError(error_msg)
            
        except Timeout as e:
            error_msg = f"Timeout when requesting {url}: {e}"
            logger.error(error_msg)
            raise WebRTCHttpError(error_msg)
            
        except HTTPError as e:
            error_msg = f"HTTP error {response.status_code} when requesting {url}: {e}"
            logger.error(error_msg)
            raise WebRTCHttpError(error_msg)
            
        except RequestException as e:
            error_msg = f"Request error when requesting {url}: {e}"
            logger.error(error_msg)
            raise WebRTCHttpError(error_msg)
    
    def get_robot_public_key(self, robot_ip: str) -> Optional[requests.Response]:
        """
        Get robot's public key for encryption.
        
        Args:
            robot_ip: Robot IP address
            
        Returns:
            Response containing encrypted public key data
        """
        url = f"http://{robot_ip}:9991/con_notify"
        
        try:
            return self.make_request(url, method='POST')
        except WebRTCHttpError as e:
            logger.error(f"Failed to get robot public key: {e}")
            raise
    
    def send_encrypted_sdp(
        self, 
        robot_ip: str, 
        path_ending: str, 
        encrypted_data: Dict[str, str]
    ) -> Optional[requests.Response]:
        """
        Send encrypted SDP offer to robot.
        
        Args:
            robot_ip: Robot IP address
            path_ending: Calculated path ending for connection
            encrypted_data: Dictionary containing encrypted SDP and AES key
            
        Returns:
            Response containing encrypted SDP answer
        """
        url = f"http://{robot_ip}:9991/con_ing_{path_ending}"
        headers = {'Content-Type': 'application/x-www-form-urlencoded'}
        
        try:
            import json
            body = json.dumps(encrypted_data)
            return self.make_request(url, method='POST', body=body, headers=headers)
        except WebRTCHttpError as e:
            logger.error(f"Failed to send encrypted SDP: {e}")
            raise
    
    def close(self):
        """Close the HTTP session"""
        if self.session:
            self.session.close()
    
    def __enter__(self):
        """Context manager entry"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.close()


# Backward compatibility function
def make_local_request(path: str, body: Optional[str] = None, headers: Optional[Dict[str, str]] = None) -> Optional[requests.Response]:
    """
    Legacy function for backward compatibility.
    
    Args:
        path: Request URL
        body: Request body
        headers: Request headers
        
    Returns:
        Response object or None
    """
    try:
        with HttpClient() as client:
            return client.make_request(path, body=body, headers=headers)
    except WebRTCHttpError:
        return None 