#pragma once

#include "BVolume.h"

template<class T, int Size>
class TAABB :public TBVolume < T, Size >
{
public:

	TVec<T, Size> border[2]; //0-min 1-max

	TVec<T, Size> GetLowerBound()
	{
		return border[0];
	}
	void SetLowerBound(TVec<T, Size> value)
	{
		border[0] = value;
	}

	TVec<T, Size> GetUpperBound()
	{
		return border[0];
	}
	void SetUpperBound(TVec<T, Size> value)
	{
		border[1] = value;
	}

	TAABB(){}
	TAABB(const TVec<T, Size>& use_pos, const TVec<T, Size>& half_size)
	{
		border[0] = use_pos - half_size;
		border[1] = use_pos + half_size;
	}
	TVec<T, Size> GetHalfSize()const								{ return (border[1] - border[0])*0.5; }
	TVec<T, Size> GetSize()const								{ return (border[1] - border[0]); }
	TVec<T, Size> GetPosition()const								{ return GetCenter(); }
	TVec<T, Size> GetCenter()const								{ return (border[1] + border[0])*0.5; }
	void Set(int use_bound, int use_dim, T use_val)				{ border[use_bound][use_dim] = use_val; }
	TVec<T, Size> operator[](int use_bound)const					{ return border[use_bound]; }

	void operator+=(const TVec<T, Size>& pos)
	{
		for (int k = 0; k<Size; k++)
		{
			if (pos[k]>border[1][k])border[1][k] = pos[k];
			else if (pos[k] < border[0][k])border[0][k] = pos[k];
		}
	}
	void operator+=(const TAABB<T, Size>& box)
	{
		for (int k = 0; k<Size; k++)
		{
			if (box.border[1][k]>border[1][k])border[1][k] = box.border[1][k];
			else if (box.border[0][k] < border[0][k])border[0][k] = box.border[0][k];
		}
	}
	void Extend(TVec<T, Size> v)
	{
		TVec<T, Size> ext(v);
		border[0] -= ext;
		border[1] += ext;
	}
	//generic for trees
	void ToSubCube(TVec<int,Size> sc) //i - x 0 min 1 max    k - y min max   t - z min max
	{
		for (int i = 0; i < Size; i++)
		{
			if (sc[i] == 0)
				border[1][i] = (border[0][i] + border[1][i])*0.5f;
			else	
				border[0][i] = (border[0][i] + border[1][i])*0.5f;
		}
	}

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

	virtual void DrawTriangles(std::vector<TVec<T, Size> >& vertices, std::vector<unsigned int>& indices)const;
	virtual void DrawLines(std::vector<TVec<T, Size> >& vertices)const;

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

typedef TAABB<float, 2>			TAABB2;