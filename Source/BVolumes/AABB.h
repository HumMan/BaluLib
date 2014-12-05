#pragma once

#include "BVolume.h"

template<class T, int Size>
class TAABB :public TBVolume < T, Size >
{
public:

	TVec<T, Size> border[2]; //0-min 1-max

	TAABB(){}
	TAABB(const TVec<T, Size>& use_pos, const TVec<T, Size>& use_widths)
	{
		border[0] = use_pos - use_widths;
		border[1] = use_pos + use_widths;
	}
	TVec<T, Size> GetSize()const									{ return (border[1] - border[0])*0.5; }
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
	void Extend(TVec<T, Size> v)
	{
		TVec<T, Size> ext(v);
		border[0] -= ext;
		border[1] += ext;
	}
	//generic for trees
	void ToSubCube(int i, int k) //i - x min max    k - y min max
	{
		static_assert(Size == 2, "only 2d support");
		if (i == 0)border[1][0] = (border[0][0] + border[1][0])*0.5f;
		else	border[0][0] = (border[0][0] + border[1][0])*0.5f;
		if (k == 0)border[1][1] = (border[0][1] + border[1][1])*0.5f;
		else	border[0][1] = (border[0][1] + border[1][1])*0.5f;
	}
	void ToSubCube(int i, int k, int t)
	{
		static_assert(Size == 3, "only 3d support");
		if (i == 0)border[1][0] = (border[0][0] + border[1][0])*0.5f;
		else	border[0][0] = (border[0][0] + border[1][0])*0.5f;
		if (k == 0)border[1][1] = (border[0][1] + border[1][1])*0.5f;
		else	border[0][1] = (border[0][1] + border[1][1])*0.5f;
		if (t == 0)border[1][2] = (border[0][2] + border[1][2])*0.5f;
		else	border[0][2] = (border[0][2] + border[1][2])*0.5f;
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

	virtual bool CollideWith(const TBVolume<T, Size>& v)const;
	virtual bool CollideWith(const TFrustum<T, Size>& frustum)const;
	virtual bool CollideWith(const TFrustum<T, Size>& frustum, bool& full_in_frustum)const;
	virtual bool CollideWith(const TAABB<T, Size>& v)const;
	virtual bool CollideWith(const TOBB<T, Size>& v)const;
	virtual bool CollideWith(const TCapsule<T, Size>& v)const;
	virtual bool CollideWith(const TSphere<T, Size>& v)const;
};
