# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
DEPRECATED: This file has been moved to infrastructure layer.
Use imports from go2_robot_sdk.infrastructure.webrtc instead.
This compatibility layer will be removed in future versions.

Originally forked from https://github.com/tfoldi/go2-webrtc and 
https://github.com/legion1581/go2_webrtc_connect
Big thanks to @tfoldi (Földi Tamás) and @legion1581 (The RoboVerse Discord Group)
"""

import warnings
from go2_robot_sdk.infrastructure.webrtc import (
    Go2Connection, make_local_request, deal_array_buffer,
    CryptoUtils, ValidationCrypto, PathCalculator
)

# Issue deprecation warning
warnings.warn(
    "scripts.webrtc_driver is deprecated. Use go2_robot_sdk.infrastructure.webrtc instead.",
    DeprecationWarning,
    stacklevel=2
)

# Re-export main classes and functions for backward compatibility
__all__ = [
    'Go2Connection', 'make_local_request', 'deal_array_buffer',
    'calc_local_path_ending', 'generate_aes_key', 'rsa_load_public_key',
    'pad', 'unpad', 'aes_encrypt', 'aes_decrypt', 'rsa_encrypt',
    'hex_to_base64', 'encrypt_key', 'encrypt_by_md5'
]

# Function aliases for backward compatibility
calc_local_path_ending = PathCalculator.calc_local_path_ending
generate_aes_key = CryptoUtils.generate_aes_key
rsa_load_public_key = CryptoUtils.rsa_load_public_key
pad = CryptoUtils.pad
unpad = CryptoUtils.unpad
aes_encrypt = CryptoUtils.aes_encrypt
aes_decrypt = CryptoUtils.aes_decrypt
rsa_encrypt = CryptoUtils.rsa_encrypt
hex_to_base64 = ValidationCrypto.hex_to_base64
encrypt_key = ValidationCrypto.encrypt_key
encrypt_by_md5 = ValidationCrypto.encrypt_by_md5
