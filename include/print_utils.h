#pragma once

#include <stdio.h>

//Unless specifically told otherwise, we will print only Critical warnings
#ifndef VERBOSITY
  #define VERBOSITY 1
#endif

// DEBUG_PRINT should be used frequently enough that we can trace the program execution
// state from looking at the debug output.
#if defined(VERBOSITY) && VERBOSITY >= 3
  #define DEBUG_PRINT(fmt, args...) fprintf(stderr, "DEBUG: %s:%s():%d: " fmt, \
    __FILE__, __func__,  __LINE__, ##args)
#else
  #define DEBUG_PRINT(fmt, args...)
#endif

// WARN_PRINT should be used to alert the user that something "weird" happened,
// when it is not critical enough to terminate the program. Most helpful
// during development
#if defined(VERBOSITY) && VERBOSITY >= 2
  #define WARN_PRINT(fmt, args...) fprintf(stderr, "WARNING: %s:%s():%d: " fmt, \
    __FILE__, __func__, __LINE__, ##args)
#else
  #define WARN_PRINT(fmt, args...)
#endif


// CRITICAL_PRINT is for unrecoverable errors 
#if defined(VERBOSITY) && VERBOSITY >= 1
  #define CRITICAL_PRINT(fmt, args...) fprintf(stderr, "CRITICAL: %s:%s():%d: " fmt, \
    __FILE__, __func__, __LINE__, ##args)
#else
  #define CRITICAL_PRINT(fmt, args...)
#endif

