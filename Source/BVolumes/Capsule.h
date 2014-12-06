#pragma once

#include "BVolume.h"

template<class T,int Size>
class TCapsule:public TBVolume<T,Size>
{
public:
	TSegment<T, Size> segment;
	T radius;

	TCapsule(){}
	TCapsule(const TVec<T,Size>& use_p0,const TVec<T,Size>& use_p1,T use_radius)
		:segment(use_p0,use_p1),radius(use_radius)
	{
	}
	void SetP0(const TVec<T, Size>& use_pos)					{ segment.p0 = use_pos; }
	void SetP1(const TVec<T, Size>& use_pos)					{ segment.p1 = use_pos; }
	T GetRadius()											{return radius;}
	TVec<T, Size> GetP0()									{ return segment.p0; }
	TVec<T, Size> GetP1()									{ return segment.p1; }
	void SetRadius(T use_rad)								{radius=use_rad;}

	TMatrix<T, Size> GetOrientation()const;

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
