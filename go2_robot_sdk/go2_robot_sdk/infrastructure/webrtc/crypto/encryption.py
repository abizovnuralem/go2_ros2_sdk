# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
Encryption utilities for Go2 WebRTC communication.
Handles AES and RSA encryption/decryption for secure robot communication.
Originally forked from https://github.com/tfoldi/go2-webrtc and 
https://github.com/legion1581/go2_webrtc_connect
"""

import binascii
import uuid
import base64
import hashlib
from typing import Optional
from Crypto.PublicKey import RSA
from Crypto.Cipher import AES, PKCS1_v1_5


class EncryptionError(Exception):
    """Custom exception for encryption-related errors"""
    pass


class CryptoUtils:
    """Utilities for encryption and decryption operations"""
    
    @staticmethod
    def generate_aes_key() -> str:
        """Generate a random AES key from UUID"""
        uuid_32 = uuid.uuid4().bytes
        uuid_32_hex_string = binascii.hexlify(uuid_32).decode('utf-8')
        return uuid_32_hex_string
    
    @staticmethod
    def rsa_load_public_key(pem_data: str) -> RSA.RsaKey:
        """Load an RSA public key from a PEM-formatted string"""
        try:
            key_bytes = base64.b64decode(pem_data)
            return RSA.import_key(key_bytes)
        except Exception as e:
            raise EncryptionError(f"Failed to load RSA public key: {e}")
    
    @staticmethod
    def pad(data: str) -> bytes:
        """Pad data to be a multiple of 16 bytes (AES block size)"""
        block_size = AES.block_size
        padding = block_size - len(data) % block_size
        padded_data = data + chr(padding) * padding
        return padded_data.encode('utf-8')
    
    @staticmethod
    def unpad(data: bytes) -> str:
        """Remove padding from data"""
        if not data:
            raise EncryptionError("Cannot unpad empty data")
        
        padding = data[-1]
        if padding > len(data) or padding == 0:
            raise EncryptionError("Invalid padding")
        
        return data[:-padding].decode('utf-8')
    
    @staticmethod
    def aes_encrypt(data: str, key: str) -> str:
        """Encrypt the given data using AES (ECB mode with PKCS5 padding)"""
        try:
            # Ensure key is 32 bytes for AES-256
            key_bytes = key.encode('utf-8')
            
            # Pad the data to ensure it is a multiple of block size
            padded_data = CryptoUtils.pad(data)
            
            # Create AES cipher in ECB mode
            cipher = AES.new(key_bytes, AES.MODE_ECB)
            
            encrypted_data = cipher.encrypt(padded_data)
            encoded_encrypted_data = base64.b64encode(encrypted_data).decode('utf-8')
            
            return encoded_encrypted_data
            
        except Exception as e:
            raise EncryptionError(f"Failed to encrypt data: {e}")
    
    @staticmethod
    def aes_decrypt(encrypted_data: str, key: str) -> str:
        """Decrypt the given data using AES (ECB mode with PKCS5 padding)"""
        try:
            # Ensure key is 32 bytes for AES-256
            key_bytes = key.encode('utf-8')
            
            # Decode Base64 encrypted data
            encrypted_data_bytes = base64.b64decode(encrypted_data)
            
            # Create AES cipher in ECB mode
            cipher = AES.new(key_bytes, AES.MODE_ECB)
            
            # Decrypt data
            decrypted_padded_data = cipher.decrypt(encrypted_data_bytes)
            
            # Unpad the decrypted data
            decrypted_data = CryptoUtils.unpad(decrypted_padded_data)
            
            return decrypted_data
            
        except Exception as e:
            raise EncryptionError(f"Failed to decrypt data: {e}")
    
    @staticmethod
    def rsa_encrypt(data: str, public_key: RSA.RsaKey) -> str:
        """Encrypt data using RSA and a given public key"""
        try:
            cipher = PKCS1_v1_5.new(public_key)
            
            # Maximum chunk size for encryption with RSA/ECB/PKCS1Padding is key size - 11 bytes
            max_chunk_size = public_key.size_in_bytes() - 11
            data_bytes = data.encode('utf-8')
            
            encrypted_bytes = bytearray()
            for i in range(0, len(data_bytes), max_chunk_size):
                chunk = data_bytes[i:i + max_chunk_size]
                encrypted_chunk = cipher.encrypt(chunk)
                encrypted_bytes.extend(encrypted_chunk)
            
            # Encode the encrypted data as Base64
            encoded_encrypted_data = base64.b64encode(encrypted_bytes).decode('utf-8')
            return encoded_encrypted_data
            
        except Exception as e:
            raise EncryptionError(f"Failed to RSA encrypt data: {e}")


class ValidationCrypto:
    """Handles robot validation encryption"""
    
    @staticmethod
    def hex_to_base64(hex_str: str) -> str:
        """Convert hex string to base64"""
        try:
            bytes_array = bytes.fromhex(hex_str)
            return base64.b64encode(bytes_array).decode("utf-8")
        except Exception as e:
            raise EncryptionError(f"Failed to convert hex to base64: {e}")
    
    @staticmethod
    def encrypt_key(key: str) -> str:
        """Encrypt validation key with Go2 specific format"""
        try:
            prefixed_key = f"UnitreeGo2_{key}"
            encrypted = ValidationCrypto.encrypt_by_md5(prefixed_key)
            return ValidationCrypto.hex_to_base64(encrypted)
        except Exception as e:
            raise EncryptionError(f"Failed to encrypt validation key: {e}")
    
    @staticmethod
    def encrypt_by_md5(input_str: str) -> str:
        """Create MD5 hash of input string"""
        try:
            hash_obj = hashlib.md5()
            hash_obj.update(input_str.encode("utf-8"))
            return hash_obj.hexdigest()
        except Exception as e:
            raise EncryptionError(f"Failed to create MD5 hash: {e}")


class PathCalculator:
    """Calculates connection path endings for Go2 protocol"""
    
    @staticmethod
    def calc_local_path_ending(data1: str) -> str:
        """Calculate local path ending from server response"""
        try:
            # Initialize an array of strings
            str_arr = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J"]
            
            # Extract the last 10 characters of data1
            last_10_chars = data1[-10:]
            
            # Split the last 10 characters into chunks of size 2
            chunked = [last_10_chars[i:i + 2] for i in range(0, len(last_10_chars), 2)]
            
            # Initialize an empty list to store indices
            array_list = []
            
            # Iterate over the chunks and find the index of the second character in str_arr
            for chunk in chunked:
                if len(chunk) > 1:
                    second_char = chunk[1]
                    try:
                        index = str_arr.index(second_char)
                        array_list.append(index)
                    except ValueError:
                        # Handle case where the character is not found in str_arr
                        # Following original implementation - ignore unknown characters
                        pass
            
            # Convert array_list to a string without separators
            join_to_string = ''.join(map(str, array_list))
            
            return join_to_string
            
        except Exception as e:
            raise EncryptionError(f"Failed to calculate path ending: {e}") 