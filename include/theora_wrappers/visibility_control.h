#ifndef THEORA_WRAPPERS__VISIBILITY_CONTROL_H_
#define THEORA_WRAPPERS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define THEORA_WRAPPERS_EXPORT __attribute__ ((dllexport))
    #define THEORA_WRAPPERS_IMPORT __attribute__ ((dllimport))
  #else
    #define THEORA_WRAPPERS_EXPORT __declspec(dllexport)
    #define THEORA_WRAPPERS_IMPORT __declspec(dllimport)
  #endif
  #ifdef THEORA_WRAPPERS_BUILDING_LIBRARY
    #define THEORA_WRAPPERS_PUBLIC THEORA_WRAPPERS_EXPORT
  #else
    #define THEORA_WRAPPERS_PUBLIC THEORA_WRAPPERS_IMPORT
  #endif
  #define THEORA_WRAPPERS_PUBLIC_TYPE THEORA_WRAPPERS_PUBLIC
  #define THEORA_WRAPPERS_LOCAL
#else
  #define THEORA_WRAPPERS_EXPORT __attribute__ ((visibility("default")))
  #define THEORA_WRAPPERS_IMPORT
  #if __GNUC__ >= 4
    #define THEORA_WRAPPERS_PUBLIC __attribute__ ((visibility("default")))
    #define THEORA_WRAPPERS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define THEORA_WRAPPERS_PUBLIC
    #define THEORA_WRAPPERS_LOCAL
  #endif
  #define THEORA_WRAPPERS_PUBLIC_TYPE
#endif

#endif  // THEORA_WRAPPERS__VISIBILITY_CONTROL_H_
