#pragma once

#include <stdio.h>
#include <stdlib.h>

#if defined _WIN32 || defined __CYGWIN__ || defined __MINGW32__
  #ifdef BALULIB_DLL_EXPORT
    #ifdef __GNUC__
      #define BALULIB_DLL_INTERFACE __attribute__ ((dllexport))
    #else
      #define BALULIB_DLL_INTERFACE __declspec(dllexport) // Note: actually gcc seems to also supports this syntax.
    #endif
  #else
    #ifdef __GNUC__
      #define BALULIB_DLL_INTERFACE __attribute__ ((dllimport))
    #else
      #define BALULIB_DLL_INTERFACE __declspec(dllimport) // Note: actually gcc seems to also supports this syntax.
    #endif
  #endif
  #define DLL_LOCAL
#else
  #if __GNUC__ >= 4
    #define BALULIB_DLL_INTERFACE __attribute__ ((visibility ("default")))
    #define DLL_LOCAL  __attribute__ ((visibility ("hidden")))
  #else
    #define BALULIB_DLL_INTERFACE
    #define DLL_LOCAL
  #endif
#endif