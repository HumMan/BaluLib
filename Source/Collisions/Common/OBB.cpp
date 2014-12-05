#include "../../BVolumes/OBB.h"

#include "../AABBAndAABB.h"
#include "../CapsuleAndOBB.h"
#include "../OBBAndOBB.h"
#include "../OBBAndSphere.h"
#include "../OBBAndAABB.h"

#include "../../BVolumes/Frustum.h"

template<class T, int Size>
bool TOBB<T, Size>::CollideWith(const TBVolume<T, Size>& v)const
{
	return v.CollideWith(*this);
}

template<class T, int Size>
bool TOBB<T, Size>::CollideWith(const TFrustum<T, Size>& frustum)const
{
	return frustum.Overlaps(*this);
}

template<class T, int Size>
bool TOBB<T, Size>::CollideWith(const TFrustum<T, Size>& frustum, bool& fully_in_frustum)const
{
	return frustum.Overlaps(*this, fully_in_frustum);
}

template<class T, int Size>
bool TOBB<T, Size>::CollideWith(const TAABB<T, Size>& v)const
{
	return Collide<T, Size>(*this, v);
}

template<class T, int Size>
bool TOBB<T, Size>::CollideWith(const TOBB<T, Size>& v)const
{
	return Collide<T, Size>(v, *this);
}

template<class T, int Size>
bool TOBB<T, Size>::CollideWith(const TCapsule<T, Size>& v)const
{
	return Collide(v, *this);
}

template<class T, int Size>
bool TOBB<T, Size>::CollideWith(const TSphere<T, Size>& v)const
{
	return Collide<T, Size>(v, *this);
}

template<class T, int Size>
bool TOBB<T, Size>::PointCollide(const TVec<T, Size>& point) const
{
	return local.PointCollide(orient.TransMul(point - pos));
}

template<class T, int Size>
bool TOBB<T, Size>::PointCollide(const TVec<T, Size>& point, TPointCollisionInfo<T, Size>& collision) const
{
	bool result = local.PointCollide(orient.TransMul(point - pos), collision);
	collision.nearest_point = orient*collision.nearest_point + pos;
	collision.normal = orient*collision.normal;
	return result;
}

template<class T, int Size>
bool TOBB<T, Size>::RayCollide(const TRay<T, Size> &ray) const
{
	return local.RayCollide(TRay<T, Size>(orient.TransMul(ray.pos - pos), orient.TransMul(ray.dir)));
}

template<class T, int Size>
bool TOBB<T, Size>::RayCollide(const TRay<T, Size> &ray, TRayCollisionInfo<T, Size>& collision) const
{
	bool result = local.RayCollide(TRay<T, Size>(orient.TransMul(ray.pos - pos), orient.TransMul(ray.dir)), collision);
	//TODO
	//normal0 = orient*normal0;
	//normal1 = orient*normal1;
	return result;
}

template<class T, int Size>
bool TOBB<T, Size>::PlaneCollide(const TPlane<T, Size> &plane) const;
template<class T, int Size>
bool TOBB<T, Size>::PlaneCollide(const TPlane<T, Size> &plane, TPlaneCollisionInfo<T, Size>& collision) const;
template<class T, int Size>
bool TOBB<T, Size>::SegmentCollide(const TSegment<T, Size> &segment) const;
template<class T, int Size>
bool TOBB<T, Size>::SegmentCollide(const TSegment<T, Size> &segment, TRayCollisionInfo<T, Size>& collision) const;
template<class T, int Size>
bool TOBB<T, Size>::LineCollide(const TLine<T, Size> &line) const;
template<class T, int Size>
bool TOBB<T, Size>::LineCollide(const TLine<T, Size> &line, TRayCollisionInfo<T, Size>& collision) const;

template<class T, int Size>
void TOBB<T, Size>::DrawTriangles(std::vector<TVec<T, Size> >& vertices, std::vector<unsigned int>& indices)const
{
	int vertices_high = vertices.size();
	local.DrawTriangles(vertices, indices);
	for (int i = vertices_high; i < vertices.size(); i++)
		vertices[i] = orient*vertices[i] + pos;
}

template<class T, int Size>
void TOBB<T, Size>::DrawLines(std::vector<TVec<T, Size> >& vertices)const
{
	int vertices_high = vertices.size();
	local.DrawLines(vertices);
	for (int i = vertices_high; i < vertices.size(); i++)
		vertices[i] = orient*vertices[i] + pos;
}

template class TOBB < float, 2 > ;
template class TOBB < float, 3 > ;
template class TOBB < double, 2 > ;
template class TOBB < double, 3 > ;