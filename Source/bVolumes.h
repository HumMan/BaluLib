#pragma once

namespace BaluLib
{

	template<class T, int size>class TAABB;
	template<class T, int size>class TOBB;
	template<class T, int size>class TSphere;
	template<class T, int size>class TCapsule;
	template<class T, int size>class TFrustum;

}
#include "BVolumes/BVolume.h"
#include "BVolumes/AABB.h"
#include "BVolumes/OBB.h"
#include "BVolumes/Sphere.h"
#include "BVolumes/Capsule.h"
#include "BVolumes/Frustum.h"