// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VISP_CAMERA_CALIBRATION__VISIBILITY_H_
#define VISP_CAMERA_CALIBRATION__VISIBILITY_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

  #ifdef __GNUC__
    #define VISP_CAMERA_CALIBRATION_EXPORT __attribute__ ((dllexport))
    #define VISP_CAMERA_CALIBRATION_IMPORT __attribute__ ((dllimport))
  #else
    #define VISP_CAMERA_CALIBRATION_EXPORT __declspec(dllexport)
    #define VISP_CAMERA_CALIBRATION_IMPORT __declspec(dllimport)
  #endif

  #ifdef VISP_CAMERA_CALIBRATION_DLL
    #define VISP_CAMERA_CALIBRATION_PUBLIC VISP_CAMERA_CALIBRATION_EXPORT
  #else
    #define VISP_CAMERA_CALIBRATION_PUBLIC VISP_CAMERA_CALIBRATION_IMPORT
  #endif

  #define VISP_CAMERA_CALIBRATION_PUBLIC_TYPE VISP_CAMERA_CALIBRATION_PUBLIC

  #define VISP_CAMERA_CALIBRATION_LOCAL

#else

  #define VISP_CAMERA_CALIBRATION_EXPORT __attribute__ ((visibility("default")))
  #define VISP_CAMERA_CALIBRATION_IMPORT

  #if __GNUC__ >= 4
    #define VISP_CAMERA_CALIBRATION_PUBLIC __attribute__ ((visibility("default")))
    #define VISP_CAMERA_CALIBRATION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define VISP_CAMERA_CALIBRATION_PUBLIC
    #define VISP_CAMERA_CALIBRATION_LOCAL
  #endif

  #define VISP_CAMERA_CALIBRATION_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // VISP_CAMERA_CALIBRATION__VISIBILITY_H_
