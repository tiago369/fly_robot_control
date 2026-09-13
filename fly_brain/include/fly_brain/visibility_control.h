// Standard Windows/GCC symbol-visibility boilerplate for ros2_control
// pluginlib-loaded controller libraries. Placeholder for M2-M4; each
// controller header will use FLY_BRAIN_PUBLIC on its exported class.
#ifndef FLY_BRAIN__VISIBILITY_CONTROL_H_
#define FLY_BRAIN__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
#ifdef FLY_BRAIN_BUILDING_DLL
#define FLY_BRAIN_PUBLIC __declspec(dllexport)
#else
#define FLY_BRAIN_PUBLIC __declspec(dllimport)
#endif
#else
#define FLY_BRAIN_PUBLIC __attribute__((visibility("default")))
#endif

#endif  // FLY_BRAIN__VISIBILITY_CONTROL_H_
