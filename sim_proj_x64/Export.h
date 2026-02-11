#pragma once

#if defined(_WIN32) || defined(__CYGWIN__)

#ifdef COPTER_EXPORTS
#define COPTER_API __declspec(dllexport)
#else
#define COPTER_API __declspec(dllimport)
#endif

#else

// Linux / macOS / gcc / clang
#define COPTER_API __attribute__((visibility("default")))

#endif
