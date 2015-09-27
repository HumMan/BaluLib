#pragma once

#include "..\primitives.h"
namespace BaluLib
{

	template < class T, int Size >
	struct TRayCollisionInfo
	{
		bool have_in;
		TVec<T, Size> in_pos, in_normal;
		T in_param;
		bool have_out;
		TVec<T, Size> out_pos, out_normal;
		T out_param;
		//TVec<T, Size> param_direction;
	};

	template < class T, int Size >
	struct TPointCollisionInfo
	{
		bool is_in;
		T distance;
		TVec<T, Size> nearest_point;
		TVec<T, Size> normal;
	};

	template < class T, int Size >
	struct TPlaneCollisionInfo
	{
		T distance;
		TVec<T, Size> nearest_point;
		TVec<T, Size> normal;
		TVec<T, Size> plane_point;
	};

	template<class T, int Size>
	class TFrustum;
	template<class T, int Size>
	class TAABB;
	template<class T, int Size>
	class TOBB;
	template<class T, int Size>
	class TCapsule;
	template<class T, int Size>
	class TSphere;

	template<class T, int Size>
	class TBVolume
	{
	public:

		virtual bool PointCollide(const TVec<T, Size>& point) const = 0;
		virtual bool PointCollide(const TVec<T, Size>& point, TPointCollisionInfo<T, Size>& collision) const = 0;
		virtual bool RayCollide(const TRay<T, Size> &ray) const = 0;
		virtual bool RayCollide(const TRay<T, Size> &ray, TRayCollisionInfo<T, Size>& collision) const = 0;
		virtual bool PlaneCollide(const TPlane<T, Size> &plane) const = 0;
		virtual bool PlaneCollide(const TPlane<T, Size> &plane, TPlaneCollisionInfo<T, Size>& collision) const = 0;
		virtual bool SegmentCollide(const TSegment<T, Size> &segment) const = 0;
		virtual bool SegmentCollide(const TSegment<T, Size> &segment, TRayCollisionInfo<T, Size>& collision) const = 0;
		virtual bool LineCollide(const TLine<T, Size> &line) const = 0;
		virtual bool LineCollide(const TLine<T, Size> &line, TRayCollisionInfo<T, Size>& collision) const = 0;

		virtual void DrawTriangles(std::vector<TVec<T, Size> >& vertices, std::vector<unsigned int>& indices) const = 0;
		virtual void DrawLines(std::vector<TVec<T, Size> >& vertices) const = 0;

		//
		virtual bool CollideWith(const TBVolume<T, Size>& volume) const = 0;
		virtual bool CollideWith(const TBVolume<T, Size>& volume, bool& fully_in_volume) const = 0;
		virtual bool CollideWith(const TFrustum<T, Size>& frustum) const = 0;
		virtual bool CollideWith(const TFrustum<T, Size>& frustum, bool& fully_in_frustum) const = 0;
		virtual bool CollideWith(const TAABB<T, Size>& aabb) const = 0;
		virtual bool CollideWith(const TAABB<T, Size>& aabb, bool& fully_in_aabb) const = 0;
		virtual bool CollideWith(const TOBB<T, Size>& obb) const = 0;
		virtual bool CollideWith(const TOBB<T, Size>& obb, bool& fully_in_obb) const = 0;
		virtual bool CollideWith(const TCapsule<T, Size>& capsule) const = 0;
		virtual bool CollideWith(const TCapsule<T, Size>& capsule, bool& fully_in_capsule) const = 0;
		virtual bool CollideWith(const TSphere<T, Size>& sphere) const = 0;
		virtual bool CollideWith(const TSphere<T, Size>& sphere, bool& fully_in_sphere) const = 0;

	};
}