/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Eurotec, Netherlands
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

#ifndef POINTCLOUD_TO_LASERSCAN__VISIBILITY_CONTROL_H_
#define POINTCLOUD_TO_LASERSCAN__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define POINTCLOUD_TO_LASERSCAN_EXPORT __attribute__ ((dllexport))
    #define POINTCLOUD_TO_LASERSCAN_IMPORT __attribute__ ((dllimport))
  #else
    #define POINTCLOUD_TO_LASERSCAN_EXPORT __declspec(dllexport)
    #define POINTCLOUD_TO_LASERSCAN_IMPORT __declspec(dllimport)
  #endif
  #ifdef POINTCLOUD_TO_LASERSCAN_BUILDING_DLL
    #define POINTCLOUD_TO_LASERSCAN_PUBLIC POINTCLOUD_TO_LASERSCAN_EXPORT
  #else
    #define POINTCLOUD_TO_LASERSCAN_PUBLIC POINTCLOUD_TO_LASERSCAN_IMPORT
  #endif
  #define POINTCLOUD_TO_LASERSCAN_PUBLIC_TYPE POINTCLOUD_TO_LASERSCAN_PUBLIC
  #define POINTCLOUD_TO_LASERSCAN_LOCAL
#else
  #define POINTCLOUD_TO_LASERSCAN_EXPORT __attribute__ ((visibility("default")))
  #define POINTCLOUD_TO_LASERSCAN_IMPORT
  #if __GNUC__ >= 4
    #define POINTCLOUD_TO_LASERSCAN_PUBLIC __attribute__ ((visibility("default")))
    #define POINTCLOUD_TO_LASERSCAN_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define POINTCLOUD_TO_LASERSCAN_PUBLIC
    #define POINTCLOUD_TO_LASERSCAN_LOCAL
  #endif
  #define POINTCLOUD_TO_LASERSCAN_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // POINTCLOUD_TO_LASERSCAN__VISIBILITY_CONTROL_H_
