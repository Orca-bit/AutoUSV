#ifndef NMPC_CONTROLLER_NODES__VISIBILITY_CONTROL_HPP_
#define NMPC_CONTROLLER_NODES__VISIBILITY_CONTROL_HPP_

#if defined(__WIN32)
  #if defined(NMPC_CONTROLLER_NODES_BUILDING_DLL) || defined(NMPC_CONTROLLER_NODES_EXPORTS)
    #define NMPC_CONTROLLER_NODES_PUBLIC __declspec(dllexport)
    #define NMPC_CONTROLLER_NODES_LOCAL
  #else  // defined(NMPC_CONTROLLER_NODES_BUILDING_DLL) || defined(NMPC_CONTROLLER_NODES_EXPORTS)
    #define NMPC_CONTROLLER_NODES_PUBLIC __declspec(dllimport)
    #define NMPC_CONTROLLER_NODES_LOCAL
  #endif  // defined(NMPC_CONTROLLER_NODES_BUILDING_DLL) || defined(NMPC_CONTROLLER_NODES_EXPORTS)
#elif defined(__linux__)
  #define NMPC_CONTROLLER_NODES_PUBLIC __attribute__((visibility("default")))
  #define NMPC_CONTROLLER_NODES_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define NMPC_CONTROLLER_NODES_PUBLIC __attribute__((visibility("default")))
  #define NMPC_CONTROLLER_NODES_LOCAL __attribute__((visibility("hidden")))
#else  // defined(_LINUX)
  #error "Unsupported Build Configuration"
#endif  // defined(_WINDOWS)

#endif  // NMPC_CONTROLLER_NODES__VISIBILITY_CONTROL_HPP_
