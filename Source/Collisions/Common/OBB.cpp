#include "../../BVolumes/OBB.h"

using namespace BaluLib;

#include "../AABBAndAABB.h"
#include "../CapsuleAndOBB.h"
#include "../OBBAndOBB.h"
#include "../OBBAndSphere.h"
#include "../OBBAndAABB.h"

#include "../../BVolumes/Frustum.h"
namespace BaluLib
{
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
		if (collision.have_in)
		{
			collision.in_normal = orient*collision.in_normal;
			collision.in_pos = orient*collision.in_pos + pos;
		}
		if (collision.have_out)
		{
			collision.out_normal = orient*collision.out_normal;
			collision.out_pos = orient*collision.out_pos + pos;
		}
		return result;
	}
	template<class T, int Size>
	bool TOBB<T, Size>::PlaneCollide(const TPlane<T, Size> &plane) const
	{
		TVec<T, Size> new_normal = orient.TransMul(plane.normal);
		TVec<T, Size> new_pos = orient.TransMul(plane.GetPos() - pos);
		return local.PlaneCollide(TPlane<T, Size>(new_pos, new_normal));
	}
	template<class T, int Size>
	bool TOBB<T, Size>::PlaneCollide(const TPlane<T, Size> &plane, TPlaneCollisionInfo<T, Size>& collision) const
	{
		//преобразуем плоскость из глобальной СК в СК OBB
		TVec<T, Size> new_normal = orient.TransMul(plane.normal);
		TVec<T, Size> new_pos = orient.TransMul(plane.GetPos() - pos);
		bool result = local.PlaneCollide(TPlane<T, Size>(new_pos, new_normal));
		//результаты в СК OBB, необходимо преобразовать в глобальную СК
		collision.nearest_point = orient* collision.nearest_point + pos;
		collision.plane_point = orient* collision.plane_point + pos;
		collision.normal = orient * collision.normal;
		return result;
	}
	template<class T, int Size>
	bool TOBB<T, Size>::SegmentCollide(const TSegment<T, Size> &segment) const
	{
		TVec<T, Size> p0, p1;
		p0 = orient.TransMul(segment.p0 - pos);
		p1 = orient.TransMul(segment.p1 - pos);
		return local.SegmentCollide(TSegment<T, Size>(p0, p1));
	}
	template<class T, int Size>
	bool TOBB<T, Size>::SegmentCollide(const TSegment<T, Size> &segment, TRayCollisionInfo<T, Size>& collision) const
	{
		TVec<T, Size> p0, p1;
		p0 = orient.TransMul(segment.p0 - pos);
		p1 = orient.TransMul(segment.p1 - pos);
		bool result = local.SegmentCollide(TSegment<T, Size>(p0, p1));
		if (collision.have_in)
		{
			collision.in_normal = orient*collision.in_normal;
			collision.in_pos = orient*collision.in_pos + pos;
		}
		if (collision.have_out)
		{
			collision.out_normal = orient*collision.out_normal;
			collision.out_pos = orient*collision.out_pos + pos;
		}
		return result;
	}
	template<class T, int Size>
	bool TOBB<T, Size>::LineCollide(const TLine<T, Size> &line) const
	{
		TVec<T, Size> new_pos, new_dir;
		new_pos = orient.TransMul(line.p0 - pos);
		new_dir = orient.TransMul(line.dir);
		return local.LineCollide(TLine<T, Size>(new_pos, new_dir));
	}
	template<class T, int Size>
	bool TOBB<T, Size>::LineCollide(const TLine<T, Size> &line, TRayCollisionInfo<T, Size>& collision) const
	{
		TVec<T, Size> new_pos, new_dir;
		new_pos = orient.TransMul(line.p0 - pos);
		new_dir = orient.TransMul(line.dir);
		bool result = local.LineCollide(TLine<T, Size>(new_pos, new_dir));
		if (collision.have_in)
		{
			collision.in_normal = orient*collision.in_normal;
			collision.in_pos = orient*collision.in_pos + pos;
		}
		if (collision.have_out)
		{
			collision.out_normal = orient*collision.out_normal;
			collision.out_pos = orient*collision.out_pos + pos;
		}
		return result;
	}

	template<class T, int Size>
	void TOBB<T, Size>::DrawTriangles(std::vector<TVec<T, Size> >& vertices, std::vector<unsigned int>& indices)const
	{
		int vertices_high = vertices.size();
		local.DrawTriangles(vertices, indices);
		for (size_t i = vertices_high; i < vertices.size(); i++)
			vertices[i] = orient*vertices[i] + pos;
	}

	template<class T, int Size>
	void TOBB<T, Size>::DrawLines(std::vector<TVec<T, Size> >& vertices)const
	{
		int vertices_high = vertices.size();
		local.DrawLines(vertices);
		for (size_t i = vertices_high; i < vertices.size(); i++)
			vertices[i] = orient*vertices[i] + pos;
	}



	template<class T, int Size>
	bool TOBB<T, Size>::CollideWith(const TBVolume<T, Size>& v)const
	{
		return v.CollideWith(*this);
	}
	template<class T, int Size>
	bool TOBB<T, Size>::CollideWith(const TBVolume<T, Size>& v, bool& fully_in_volume)const
	{
		return v.CollideWith(*this, fully_in_volume);
	}

	template<class T>
	bool CollideWithSpecialized(const TOBB<T, 2>& aabb, const TFrustum<T, 2>& frustum)
	{
		//static_assert(false, "supports only 3d");
		assert(false);
		return false;
	}
	template<class T>
	bool CollideWithSpecialized(const TOBB<T, 3>& aabb, const TFrustum<T, 3>& frustum)
	{
		return frustum.Overlaps(aabb);
	}
	template<class T, int Size>
	bool TOBB<T, Size>::CollideWith(const TFrustum<T, Size>& frustum)const
	{
		return CollideWithSpecialized(*this, frustum);
	}

	template<class T>
	bool CollideWithSpecialized(const TOBB<T, 2>& aabb, const TFrustum<T, 2>& frustum, bool& fully_in_frustum)
	{
		//static_assert(false, "supports only 3d");
		assert(false);
		return false;
	}
	template<class T>
	bool CollideWithSpecialized(const TOBB<T, 3>& aabb, const TFrustum<T, 3>& frustum, bool& fully_in_frustum)
	{
		return frustum.Overlaps(aabb, fully_in_frustum);
	}
	template<class T, int Size>
	bool TOBB<T, Size>::CollideWith(const TFrustum<T, Size>& frustum, bool& fully_in_frustum)const
	{
		return CollideWithSpecialized(*this, frustum, fully_in_frustum);
	}

	template<class T, int Size>
	bool TOBB<T, Size>::CollideWith(const TAABB<T, Size>& v)const
	{
		return Collide<T, Size>(*this, v);
	}
	template<class T, int Size>
	bool TOBB<T, Size>::CollideWith(const TAABB<T, Size>& v, bool& fully_in_aabb)const
	{
		return Collide<T, Size>(*this, v, fully_in_aabb);
	}
	template<class T, int Size>
	bool TOBB<T, Size>::CollideWith(const TOBB<T, Size>& v)const
	{
		return Collide<T, Size>(v, *this);
	}
	template<class T, int Size>
	bool TOBB<T, Size>::CollideWith(const TOBB<T, Size>& v, bool& fully_in_obb)const
	{
		return Collide<T, Size>(*this, v, fully_in_obb);
	}
	template<class T, int Size>
	bool TOBB<T, Size>::CollideWith(const TCapsule<T, Size>& v)const
	{
		return Collide(v, *this);
	}
	template<class T, int Size>
	bool TOBB<T, Size>::CollideWith(const TCapsule<T, Size>& v, bool& fully_in_capsule)const
	{
		return Collide(*this, v, fully_in_capsule);
	}
	template<class T, int Size>
	bool TOBB<T, Size>::CollideWith(const TSphere<T, Size>& v)const
	{
		return Collide<T, Size>(v, *this);
	}
	template<class T, int Size>
	bool TOBB<T, Size>::CollideWith(const TSphere<T, Size>& v, bool& fully_in_sphere)const
	{
		return Collide<T, Size>(*this, v, fully_in_sphere);
	}

	template class TOBB < float, 2 >;
	template class TOBB < float, 3 >;
	template class TOBB < double, 2 >;
	template class TOBB < double, 3 >;
}