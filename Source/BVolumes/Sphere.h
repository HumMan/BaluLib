#pragma once

#include "BVolume.h"

template<class T,int Size>
class TSphere :public TBVolume < T, Size >
{
public:
	TVec<T, Size> pos;
	T radius;

	TSphere(){}
	TSphere(const TVec<T, Size>& use_pos, T use_radius) :pos(use_pos), radius(use_radius){}
	TSphere(T use_radius) :radius(use_radius)				{}
	void SetPos(const TVec<T, Size>& use_pos)				{ pos = use_pos; }
	void SetRadius(T use_rad)								{ radius = use_rad; }
	TAABB<T, Size> GetAABB()const							{ return TAABB<T, Size>(pos, TVec<T,Size>(radius * 2)); }
	T GetRadius()											{ return radius; }
	TVec<T, Size> GetPos()									{ return pos; }

	//Common virtual methods
	virtual bool PointCollide(const TVec<T, Size>& point) const;
	virtual bool PointCollide(const TVec<T, Size>& point, TPointCollisionInfo<T, Size>& collision) const;
	virtual bool RayCollide(const TRay<T, Size> &ray) const;
	virtual bool RayCollide(const TRay<T, Size> &ray, TRayCollisionInfo<T, Size>& collision) const;
	virtual bool PlaneCollide(const TPlane<T, Size> &plane) const;
	virtual bool PlaneCollide(const TPlane<T, Size> &plane, TPlaneCollisionInfo<T, Size>& collision) const;
	virtual bool SegmentCollide(const TSegment<T, Size> &segment) const;
	virtual bool SegmentCollide(const TSegment<T, Size> &segment, TRayCollisionInfo<T, Size>& collision) const;
	virtual bool LineCollide(const TLine<T, Size> &line) const;
	virtual bool LineCollide(const TLine<T, Size> &line, TRayCollisionInfo<T, Size>& collision) const;

	virtual void DrawTriangles(std::vector<TVec<T, Size> >& vertices, std::vector<unsigned int>& indices) const;
	virtual void DrawLines(std::vector<TVec<T, Size> >& vertices) const;

	//
	virtual bool CollideWith(const TBVolume<T, Size>& volume) const;
	virtual bool CollideWith(const TBVolume<T, Size>& volume, bool& fully_in_volume) const;
	virtual bool CollideWith(const TFrustum<T, Size>& frustum) const;
	virtual bool CollideWith(const TFrustum<T, Size>& frustum, bool& fully_in_frustum) const;
	virtual bool CollideWith(const TAABB<T, Size>& aabb) const;
	virtual bool CollideWith(const TAABB<T, Size>& aabb, bool& fully_in_aabb) const;
	virtual bool CollideWith(const TOBB<T, Size>& obb) const;
	virtual bool CollideWith(const TOBB<T, Size>& obb, bool& fully_in_obb) const;
	virtual bool CollideWith(const TCapsule<T, Size>& capsule) const;
	virtual bool CollideWith(const TCapsule<T, Size>& capsule, bool& fully_in_capsule) const;
	virtual bool CollideWith(const TSphere<T, Size>& sphere) const;
	virtual bool CollideWith(const TSphere<T, Size>& sphere, bool& fully_in_sphere) const;
};

template<class T, int Size>
bool SphereRayCollide(const TSphere<T, Size>& sphere, const TRay<T, Size> &ray, T& t0, T& t1);