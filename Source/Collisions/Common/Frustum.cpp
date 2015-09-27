#include "../../BVolumes/Frustum.h"

namespace BaluLib
{
	template<class T, int Size>
	TFrustum<T, Size>::TFrustum(const TMatrix<T, 4>& clip)
	{
		//извлечение уравнений плоскостей из матрицы проекции
		//http://gamedevs.org/uploads/fast-extraction-viewing-frustum-planes-from-world-view-projection-matrix.pdf

		static_assert(Size == 3, "supports only 3d");
		TVec<T, 4> v0, v1;
		v0 = TVec<T, 4>(clip[0][3],
			clip[1][3],
			clip[2][3],
			clip[3][3]);

		//right, left common
		v1 = TVec<T, 4>(clip[0][0],
			clip[1][0],
			clip[2][0],
			clip[3][0]);
		//left
		frustum[0] = TPlane<T, 3>(v0 + v1);
		//right
		frustum[1] = TPlane<T, 3>(v0 - v1);

		//bottom, top common
		v1 = TVec<T, 4>(clip[0][1],
			clip[1][1],
			clip[2][1],
			clip[3][1]);

		//bottom
		frustum[2] = TPlane<T, 3>(v0 + v1);
		//top
		frustum[3] = TPlane<T, 3>(v0 - v1);

		//near, far common
		v1 = TVec<T, 4>(clip[0][2],
			clip[1][2],
			clip[2][2],
			clip[3][2]);
		//near
		frustum[4] = TPlane<T, 3>(v0 + v1);
		//far
		frustum[5] = TPlane<T, 3>(v0 - v1);

	}

	template<class T, int Size>
	bool TFrustum<T, Size>::Overlaps(const TAABB<T, Size>& use_aabb) const
	{
		TVec<T, Size> size, pos;
		pos = use_aabb.GetCenter();
		size = use_aabb.GetHalfSize();
		for (int i = 0; i < planes_count; i++)
		{
			T dist = frustum[i].DistanceTo(pos),
				abs_sc_mul = size.AbsScalarMul(frustum[i].normal);
			if (dist < -abs_sc_mul)
				return false;
		}
		return true;
	}

	template<class T, int Size>
	bool TFrustum<T, Size>::Overlaps(const TOBB<T, Size>& use_obb) const
	{
		TVec<T, Size> s(use_obb.local.GetHalfSize());
		for (int i = 0; i < planes_count; i++)
		{
			if (frustum[i].DistanceTo(use_obb.pos) <
				-s.AbsScalarMul(use_obb.orient*frustum[i].normal))
				return false;
		}
		return true;
	}

	template<class T, int Size>
	bool TFrustum<T, Size>::Overlaps(const TSphere<T, Size>& sphere) const
	{
		int c = 0;
		for (int i = 0; i < planes_count; i++){
			if ((frustum[i].DistanceTo(sphere.pos)) < -sphere.radius)
				return false;
		}
		return true;
	}

	template<class T, int Size>
	bool TFrustum<T, Size>::Overlaps(const TCapsule<T, Size>& capsule) const
	{
		int c = 0;
		for (int i = 0; i < planes_count; i++){
			if (frustum[i].DistanceTo(capsule.segment.p0) < -capsule.radius&&
				frustum[i].DistanceTo(capsule.segment.p1) < -capsule.radius)
				return false;
		}
		return true;
	}


	template<class T, int Size>
	bool TFrustum<T, Size>::Overlaps(const TAABB<T, Size>& use_aabb, bool& fully_in_frustum) const
	{
		T s, d;
		int c = 0;
		TVec<T, Size> size, pos;
		pos = use_aabb.GetCenter();
		size = use_aabb.GetHalfSize();
		//float radius=size.SqrLength();
		for (int i = 0; i < planes_count; i++)
		{
			s = size.AbsScalarMul(frustum[i].normal);
			d = frustum[i].DistanceTo(pos);
			if (d < -s)return false;
			else if (d >= s)c++;
		}
		fully_in_frustum = (c == 6);
		return true;
	}

	template<class T, int Size>
	bool TFrustum<T, Size>::Overlaps(const TOBB<T, Size> &use_obb, bool& fully_in_frustum) const
	{
		T s, d;
		TVec<T, Size> obb_size(use_obb.local.GetHalfSize());
		int c = 0;
		for (int i = 0; i < planes_count; i++)
		{
			s = obb_size.AbsScalarMul(use_obb.orient*frustum[i].normal);
			d = frustum[i].DistanceTo(use_obb.pos);
			if (d < -s)return false;
			else if (d >= s)c++;
		}
		fully_in_frustum = (c == 6);
		return true;
	}

	template<class T, int Size>
	bool TFrustum<T, Size>::Overlaps(const TSphere<T, Size>& sphere, bool& fully_in_frustum) const
	{
		int c = 0;
		T d;
		for (int i = 0; i < 6; i++)
		{
			d = frustum[i].DistanceTo(sphere.pos);
			if (d < -sphere.radius)return false;
			else if (d >= sphere.radius)c++;
		}
		fully_in_frustum = (c == 6);
		return true;
	}

	template<class T, int Size>
	bool TFrustum<T, Size>::Overlaps(const TCapsule<T, Size>& capsule, bool& fully_in_frustum) const
	{
		int c = 0;
		T d0, d1;
		for (int i = 0; i < 6; i++)
		{
			d0 = frustum[i].DistanceTo(capsule.segment.p0);
			d1 = frustum[i].DistanceTo(capsule.segment.p1);
			if (d0 < -capsule.radius&&d1 < -capsule.radius)return false;
			else if (d0 >= capsule.radius&&d1 >= capsule.radius)c++;
		}
		fully_in_frustum = (c == 6);
		return true;
	}

	template<class T, int Size>
	bool TFrustum<T, Size>::Overlaps(const TVec<T, Size>& point) const
	{
		for (int i = 0; i < planes_count; i += 2)
		{
			if (frustum[i].DistanceTo(point) < 0 || frustum[i + 1].DistanceTo(point) < 0)
				return false;
		}
		return true;
	}

	template class TFrustum<float, 3>;
	template class TFrustum<double, 3>;
}