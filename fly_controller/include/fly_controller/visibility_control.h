// Standard Windows/GCC symbol-visibility boilerplate for ros2_control
// pluginlib-loaded controller libraries (mirrors fly_brain's own
// visibility_control.h).
#ifndef FLY_CONTROLLER__VISIBILITY_CONTROL_H_
#define FLY_CONTROLLER__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
#ifdef FLY_CONTROLLER_BUILDING_DLL
#define FLY_CONTROLLER_PUBLIC __declspec(dllexport)
#else
#define FLY_CONTROLLER_PUBLIC __declspec(dllimport)
#endif
#else
#define FLY_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#endif

#endif  // FLY_CONTROLLER__VISIBILITY_CONTROL_H_
