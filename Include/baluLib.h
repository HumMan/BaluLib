#pragma once

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

inline int IntSizeOf(int x)
{
	return ((((x - 1)) / sizeof(int) + 1)*sizeof(int));
}

#include "../Source/stdfuncs.h"
#include "../Source/Math/vec.h"
#include "../Source/primitives.h"
#include "../Source/common.h"
#include "../Source/Math/matrix.h"
#include "../Source/Math/fixed.h"
#include "../Source/Math/quaternion.h"
#include "../Source/arrays.h"
//#include "../Source/bVolumes.h"
#include "../Source/camera.h"