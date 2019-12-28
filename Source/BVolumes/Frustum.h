#pragma once

#include "AABB.h"
#include "OBB.h"
#include "Sphere.h"
#include "Capsule.h"

namespace BaluLib
{

	template<class T, int Size>
	class TFrustum
	{
	public:
		enum
		{
			planes_count = Size * 2
		};
		// плоскости области видимости left, right, bottom, top, near, far
		TPlane<T, Size> frustum[planes_count];

		TFrustum(){}
		BALULIB_DLL_INTERFACE TFrustum(const TMatrix<T, 4>& clip);
		BALULIB_DLL_INTERFACE bool Overlaps(const TAABB<T, Size>& use_aabb) const;
		BALULIB_DLL_INTERFACE bool Overlaps(const TAABB<T, Size>& use_aabb, bool& fully_in_frustum) const;
		BALULIB_DLL_INTERFACE bool Overlaps(const TOBB<T, Size>& use_obb) const;
		BALULIB_DLL_INTERFACE bool Overlaps(const TOBB<T, Size>& use_obb, bool& fully_in_frustum) const;
		BALULIB_DLL_INTERFACE bool Overlaps(const TSphere<T, Size>& sphere) const;
		BALULIB_DLL_INTERFACE bool Overlaps(const TSphere<T, Size>& sphere, bool& fully_in_frustum) const;
		BALULIB_DLL_INTERFACE bool Overlaps(const TCapsule<T, Size>& sphere) const;
		BALULIB_DLL_INTERFACE bool Overlaps(const TCapsule<T, Size>& sphere, bool& fully_in_frustum) const;
		BALULIB_DLL_INTERFACE bool Overlaps(const TVec<T, Size>& point) const;
	};

}