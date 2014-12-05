#pragma once

#include "AABB.h"
#include "OBB.h"
#include "Sphere.h"
#include "Capsule.h"

template<class T, int Size>
class TFrustum
{
public:
	enum
	{
		planes_count=Size*2
	};
	// плоскости области видимости left, right, bottom, top, near, far
	TPlane<T,Size> frustum[planes_count];

	TFrustum(){}
	TFrustum(const TMatrix<T,4>& clip);
	bool Overlaps(const TAABB<T,Size>& use_aabb) const;
	bool Overlaps(const TAABB<T,Size>& use_aabb,bool& full_in_frustum) const;
	bool Overlaps(const TOBB<T,Size>& use_obb) const;
	bool Overlaps(const TOBB<T,Size>& use_obb,bool& full_in_frustum) const;
	bool Overlaps(const TSphere<T,Size>& sphere) const;
	bool Overlaps(const TSphere<T,Size>& sphere,bool& full_in_frustum) const;
	bool Overlaps(const TCapsule<T,Size>& sphere) const;
	bool Overlaps(const TCapsule<T,Size>& sphere,bool& full_in_frustum) const;
	bool Overlaps(const TVec<T,Size>& point) const;
};
