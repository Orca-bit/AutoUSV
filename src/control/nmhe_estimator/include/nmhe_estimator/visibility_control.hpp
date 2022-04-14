#ifndef NMHE_ESTIMATOR__VISIBILITY_CONTROL_HPP_
#define NMHE_ESTIMATOR__VISIBILITY_CONTROL_HPP_

#if defined(__WIN32)
  #if defined(NMHE_ESTIMATOR_BUILDING_DLL) || defined(NMHE_ESTIMATOR_EXPORTS)
    #define NMHE_ESTIMATOR_PUBLIC __declspec(dllexport)
    #define NMHE_ESTIMATOR_LOCAL
  #else  // defined(NMHE_ESTIMATOR_BUILDING_DLL) || defined(NMHE_ESTIMATOR_EXPORTS)
    #define NMHE_ESTIMATOR_PUBLIC __declspec(dllimport)
    #define NMHE_ESTIMATOR_LOCAL
  #endif  // defined(NMHE_ESTIMATOR_BUILDING_DLL) || defined(NMHE_ESTIMATOR_EXPORTS)
#elif defined(__linux__)
  #define NMHE_ESTIMATOR_PUBLIC __attribute__((visibility("default")))
  #define NMHE_ESTIMATOR_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define NMHE_ESTIMATOR_PUBLIC __attribute__((visibility("default")))
  #define NMHE_ESTIMATOR_LOCAL __attribute__((visibility("hidden")))
#else  // defined(_LINUX)
  #error "Unsupported Build Configuration"
#endif  // defined(_WINDOWS)

#endif  // NMHE_ESTIMATOR__VISIBILITY_CONTROL_HPP_
