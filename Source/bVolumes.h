#pragma once

template<class T,int size>class TAABB;
template<class T,int size>class TOBB;
template<class T,int size>class TSphere;
template<class T,int size>class TCapsule;
template<class T,int size>class TFrustum;

class TBaluRender;

//#include "Collisions\AABBAndAABB.h"
//#include "Collisions\AABBAndSphere.h"
//#include "Collisions\CapsuleAndAABB.h"
//#include "Collisions\CapsuleAndCapsule.h"
//#include "Collisions\CapsuleAndOBB.h"
//#include "Collisions\CapsuleAndSphere.h"
//#include "Collisions\OBBAndAABB.h"
//#include "Collisions\OBBAndOBB.h"
//#include "Collisions\OBBAndSphere.h"
//#include "Collisions\SphereAndSphere.h"

#include "BVolumes\BVolume.h"
#include "BVolumes\AABB.h"
#include "BVolumes\OBB.h"
#include "BVolumes\Sphere.h"
#include "BVolumes\Capsule.h"
#include "BVolumes\Frustum.h"

//#include "Collisions\Common\AABB.h"
//#include "Collisions\Common\OBB.h"
//#include "Collisions\Common\Sphere.h"
//#include "Collisions\Common\Capsule.h"