#ifndef ASTAR_SEARCH__VISIBILITY_CONTROL_HPP_
#define ASTAR_SEARCH__VISIBILITY_CONTROL_HPP_

#if defined(__WIN32)
  #if defined(ASTAR_SEARCH_BUILDING_DLL) || \
  defined(ASTAR_SEARCH_EXPORTS)
    #define ASTAR_SEARCH_PUBLIC __declspec(dllexport)
    #define ASTAR_SEARCH_LOCAL
  #else
// defined(ASTAR_SEARCH_BUILDING_DLL) || defined(ASTAR_SEARCH_EXPORTS)
    #define ASTAR_SEARCH_PUBLIC __declspec(dllimport)
    #define ASTAR_SEARCH_LOCAL
  #endif
// defined(ASTAR_SEARCH_BUILDING_DLL) || defined(ASTAR_SEARCH_EXPORTS)
#elif defined(__linux__)
  #define ASTAR_SEARCH_PUBLIC __attribute__((visibility("default")))
  #define ASTAR_SEARCH_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define ASTAR_SEARCH_PUBLIC __attribute__((visibility("default")))
  #define ASTAR_SEARCH_LOCAL __attribute__((visibility("hidden")))
#else  // defined(_LINUX)
  #error "Unsupported Build Configuration"
#endif  // defined(_WINDOWS)

#endif  // ASTAR_SEARCH__VISIBILITY_CONTROL_HPP_
