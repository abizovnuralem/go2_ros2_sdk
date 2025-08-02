# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
Geometric math utilities for Go2 robot.
Contains vector and quaternion operations for robot kinematics.
"""

import math


class Quaternion:
    """Quaternion representation for rotations"""
    
    def __init__(self, x: float, y: float, z: float, w: float):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def set_from_axis_angle(self, axis_vector: 'Vector3', angle: float) -> tuple:
        """Set quaternion from axis-angle representation"""
        half_angle = angle / 2
        sin_half = math.sin(half_angle)
        self.x = axis_vector.x * sin_half
        self.y = axis_vector.y * sin_half
        self.z = axis_vector.z * sin_half
        self.w = math.cos(half_angle)
        return self.x, self.y, self.z, self.w

    def invert(self) -> tuple:
        """Invert the quaternion (conjugate for unit quaternions)"""
        self.x *= -1
        self.y *= -1
        self.z *= -1
        return self.x, self.y, self.z, self.w


class Vector3:
    """3D Vector representation with common operations"""
    
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

    def add(self, other: 'Vector3') -> tuple:
        """Add another vector to this vector"""
        self.x += other.x
        self.y += other.y
        self.z += other.z
        return self.x, self.y, self.z

    def clone(self) -> 'Vector3':
        """Create a copy of this vector"""
        return Vector3(self.x, self.y, self.z)

    def apply_quaternion(self, quaternion: Quaternion) -> tuple:
        """Apply quaternion rotation to this vector"""
        # Quaternion-vector multiplication
        qx, qy, qz, qw = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        
        # First step: q * v
        ix = qw * self.x + qy * self.z - qz * self.y
        iy = qw * self.y + qz * self.x - qx * self.z
        iz = qw * self.z + qx * self.y - qy * self.x
        iw = -qx * self.x - qy * self.y - qz * self.z

        # Second step: result * q*
        self.x = ix * qw + iw * -qx + iy * -qz - iz * -qy
        self.y = iy * qw + iw * -qy + iz * -qx - ix * -qz
        self.z = iz * qw + iw * -qz + ix * -qy - iy * -qx
        
        return self.x, self.y, self.z

    def negate(self) -> tuple:
        """Negate all components of the vector"""
        self.x = -self.x
        self.y = -self.y
        self.z = -self.z
        return self.x, self.y, self.z

    def distance_to(self, other: 'Vector3') -> float:
        """Calculate Euclidean distance to another vector"""
        return math.sqrt(self.distance_to_squared(other))

    def distance_to_squared(self, other: 'Vector3') -> float:
        """Calculate squared distance to another vector (more efficient)"""
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return dx * dx + dy * dy + dz * dz

    def apply_axis_angle(self, axis: 'Vector3', angle: float) -> tuple:
        """Apply rotation around axis by given angle"""
        quaternion = Quaternion(0, 0, 0, 1)
        quaternion.set_from_axis_angle(axis, angle)
        return self.apply_quaternion(quaternion) 