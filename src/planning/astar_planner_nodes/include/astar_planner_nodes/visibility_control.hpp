#ifndef ASTAR_PLANNER__VISIBILITY_CONTROL_HPP_
#define ASTAR_PLANNER__VISIBILITY_CONTROL_HPP_

#if defined(__WIN32)
  #if defined(ASTAR_PLANNER_BUILDING_DLL) || \
  defined(ASTAR_PLANNER_EXPORTS)
    #define ASTAR_PLANNER_PUBLIC __declspec(dllexport)
    #define ASTAR_PLANNER_LOCAL
  #else
// defined(ASTAR_PLANNER_BUILDING_DLL) || defined(ASTAR_PLANNER_EXPORTS)
    #define ASTAR_PLANNER_PUBLIC __declspec(dllimport)
    #define ASTAR_PLANNER_LOCAL
  #endif
// defined(ASTAR_PLANNER_BUILDING_DLL) || defined(ASTAR_PLANNER_EXPORTS)
#elif defined(__linux__)
  #define ASTAR_PLANNER_PUBLIC __attribute__((visibility("default")))
  #define ASTAR_PLANNER_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define ASTAR_PLANNER_PUBLIC __attribute__((visibility("default")))
  #define ASTAR_PLANNER_LOCAL __attribute__((visibility("hidden")))
#else  // defined(_LINUX)
  #error "Unsupported Build Configuration"
#endif  // defined(_WINDOWS)

#endif  // ASTAR_PLANNER__VISIBILITY_CONTROL_HPP_
