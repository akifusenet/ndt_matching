#ifndef NDT_MATCHING__VISIBILITY_CONTROL_H_
#define NDT_MATCHING__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define NDT_MATCHING_EXPORT __attribute__ ((dllexport))
    #define NDT_MATCHING_IMPORT __attribute__ ((dllimport))
  #else
    #define NDT_MATCHING_EXPORT __declspec(dllexport)
    #define NDT_MATCHING_IMPORT __declspec(dllimport)
  #endif
  #ifdef NDT_MATCHING_BUILDING_LIBRARY
    #define NDT_MATCHING_PUBLIC NDT_MATCHING_EXPORT
  #else
    #define NDT_MATCHING_PUBLIC NDT_MATCHING_IMPORT
  #endif
  #define NDT_MATCHING_PUBLIC_TYPE NDT_MATCHING_PUBLIC
  #define NDT_MATCHING_LOCAL
#else
  #define NDT_MATCHING_EXPORT __attribute__ ((visibility("default")))
  #define NDT_MATCHING_IMPORT
  #if __GNUC__ >= 4
    #define NDT_MATCHING_PUBLIC __attribute__ ((visibility("default")))
    #define NDT_MATCHING_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define NDT_MATCHING_PUBLIC
    #define NDT_MATCHING_LOCAL
  #endif
  #define NDT_MATCHING_PUBLIC_TYPE
#endif

#endif  // NDT_MATCHING__VISIBILITY_CONTROL_H_
