#pragma once

#ifndef _MSC_VER
#define __forceinline __inline
#define _INTSIZEOF(x) ((((sizeof(x)-1))/4+1)*4)
#endif

//#include "..\Source\mballoc.h" 

//#include <iostream>
#include <stdio.h>
#include <stdlib.h>
//#include <memory.h>
//#include <string.h>
//#include <ctype.h>
#include <assert.h>
#include <vector>

#define _USE_MATH_DEFINES
#include <math.h>

//template <bool> struct CompileTimeError;
//template <> struct CompileTimeError<true>{char c;};
#ifdef COMPILE_TIME_ERR
#error "COMPILE_TIME_ERR already used"
#endif
#ifndef _DEBUG
#define COMPILE_TIME_ERR(x) ((void)0)
#else
#define COMPILE_TIME_ERR(x) {char c[x?1:-1];}
#endif


#include "../Source/stdfuncs.h"
#include "../Source/vec.h"
#include "../Source/primitives.h"
#include "../Source/common.h"
#include "../Source/matrix.h"
#include "../Source/fixed.h"
#include "../Source/quaternion.h"
#include "../Source/arrays.h"
#include "../Source/bVolumes.h"
#include "../Source/camera.h"