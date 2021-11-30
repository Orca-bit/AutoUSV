#ifndef MPC_CONTROLLER__VISIBILITY_CONTROL_HPP_
#define MPC_CONTROLLER__VISIBILITY_CONTROL_HPP_

#if defined(__WIN32)
  #if defined(MPC_CONTROLLER_BUILDING_DLL) || defined(MPC_CONTROLLER_EXPORTS)
    #define MPC_CONTROLLER_PUBLIC __declspec(dllexport)
    #define MPC_CONTROLLER_LOCAL
  #else  // defined(MPC_CONTROLLER_BUILDING_DLL) || defined(MPC_CONTROLLER_EXPORTS)
    #define MPC_CONTROLLER_PUBLIC __declspec(dllimport)
    #define MPC_CONTROLLER_LOCAL
  #endif  // defined(MPC_CONTROLLER_BUILDING_DLL) || defined(MPC_CONTROLLER_EXPORTS)
#elif defined(__linux__)
  #define MPC_CONTROLLER_PUBLIC __attribute__((visibility("default")))
  #define MPC_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define MPC_CONTROLLER_PUBLIC __attribute__((visibility("default")))
  #define MPC_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else  // defined(_LINUX)
  #error "Unsupported Build Configuration"
#endif  // defined(_WINDOWS)

#endif  // MPC_CONTROLLER__VISIBILITY_CONTROL_HPP_
