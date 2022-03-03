#ifndef GNSS_INTERFACE__VISIBILITY_CONTROL_HPP_
#define GNSS_INTERFACE__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GNSS_INTERFACE_EXPORT __attribute__ ((dllexport))
    #define GNSS_INTERFACE_IMPORT __attribute__ ((dllimport))
  #else
    #define GNSS_INTERFACE_EXPORT __declspec(dllexport)
    #define GNSS_INTERFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef GNSS_INTERFACE_BUILDING_LIBRARY
    #define GNSS_INTERFACE_PUBLIC GNSS_INTERFACE_EXPORT
  #else
    #define GNSS_INTERFACE_PUBLIC GNSS_INTERFACE_IMPORT
  #endif
  #define GNSS_INTERFACE_PUBLIC_TYPE GNSS_INTERFACE_PUBLIC
  #define GNSS_INTERFACE_LOCAL
#else
  #define GNSS_INTERFACE_EXPORT __attribute__ ((visibility("default")))
  #define GNSS_INTERFACE_IMPORT
  #if __GNUC__ >= 4
    #define GNSS_INTERFACE_PUBLIC __attribute__ ((visibility("default")))
    #define GNSS_INTERFACE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GNSS_INTERFACE_PUBLIC
    #define GNSS_INTERFACE_LOCAL
  #endif
  #define GNSS_INTERFACE_PUBLIC_TYPE
#endif

#endif  // GNSS_INTERFACE__VISIBILITY_CONTROL_HPP_
