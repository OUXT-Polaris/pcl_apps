// Copyright (c) 2019 OUXT Polaris
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

#ifndef PCL_APPS__VISIBILITY_CONTROL_H_
#define PCL_APPS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PCL_APPS_EXPORT __attribute__((dllexport))
#define PCL_APPS_IMPORT __attribute__((dllimport))
#else
#define PCL_APPS_EXPORT __declspec(dllexport)
#define PCL_APPS_IMPORT __declspec(dllimport)
#endif
#ifdef PCL_APPS_BUILDING_LIBRARY
#define PCL_APPS_PUBLIC PCL_APPS_EXPORT
#else
#define PCL_APPS_PUBLIC PCL_APPS_IMPORT
#endif
#define PCL_APPS_PUBLIC_TYPE PCL_APPS_PUBLIC
#define PCL_APPS_LOCAL
#else
#define PCL_APPS_EXPORT __attribute__((visibility("default")))
#define PCL_APPS_IMPORT
#if __GNUC__ >= 4
#define PCL_APPS_PUBLIC __attribute__((visibility("default")))
#define PCL_APPS_LOCAL __attribute__((visibility("hidden")))
#else
#define PCL_APPS_PUBLIC
#define PCL_APPS_LOCAL
#endif
#define PCL_APPS_PUBLIC_TYPE
#endif

#endif  // PCL_APPS__VISIBILITY_CONTROL_H_
