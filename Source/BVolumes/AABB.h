#pragma once

#include "BVolume.h"
namespace BaluLib
{

	template<class T, int Size>
	class TAABB :public BaluLib::TBVolume < T, Size >
	{
	public:

		TVec<T, Size> border[2]; //0-min 1-max

		T GetPerimeter() const
		{
			auto w = border[1]-border[0];
			T result = 0;
			for (int k = 0; k < Size; k++)
			{
				result += w[k];
			}
                        return (T)(result * 2.0);
		}
		bool Contains(const TAABB<T, Size>& box)const
		{
			bool fully_in_aabb;
			CollideWith(box, fully_in_aabb);
			return fully_in_aabb;
		}
		TVec<T, Size> GetLowerBound()const
		{
			return border[0];
		}
		void SetLowerBound(TVec<T, Size> value)
		{
			border[0] = value;
		}

		TVec<T, Size> GetUpperBound()const
		{
			return border[0];
		}
		void SetUpperBound(TVec<T, Size> value)
		{
			border[1] = value;
		}

		TAABB() {}
		TAABB(const TVec<T, Size>& use_pos, const TVec<T, Size>& half_size)
		{
			border[0] = use_pos - half_size;
			border[1] = use_pos + half_size;
		}
		TVec<T, Size> GetHalfSize()const { return (border[1] - border[0])*0.5; }
		TVec<T, Size> GetSize()const { return (border[1] - border[0]); }
		TVec<T, Size> GetPosition()const { return GetCenter(); }
		TVec<T, Size> GetCenter()const { return (border[1] + border[0])*0.5; }
		void Set(int use_bound, int use_dim, T use_val) { border[use_bound][use_dim] = use_val; }
		TVec<T, Size> operator[](int use_bound)const { return border[use_bound]; }
		/// Combine an position into this one
		void Combine(const TVec<T, Size>& pos)
		{
			for (int k = 0; k < Size; k++)
			{
				if (pos[k] > border[1][k])border[1][k] = pos[k];
				else if (pos[k] < border[0][k])border[0][k] = pos[k];
			}
		}
		/// Combine an AABB into this one
		void Combine(const TAABB<T, Size>& box)
		{
			for (int k = 0; k < Size; k++)
			{
				if (box.border[1][k] > border[1][k])border[1][k] = box.border[1][k];
				else if (box.border[0][k] < border[0][k])border[0][k] = box.border[0][k];
			}
		}
		/// Combine two AABB into this one
		void Combine(const TAABB<T, Size>& box0, const TAABB<T, Size>& box1)
		{
			for (int k = 0; k < Size; k++)
			{
				border[1][k] = Max(box0.border[1][k], box1.border[1][k]);
				border[2][k] = Max(box0.border[2][k], box1.border[2][k]);
			}
		}
		void Extend(TVec<T, Size> v)
		{
			TVec<T, Size> ext(v);
			border[0] -= ext;
			border[1] += ext;
		}
		//generic for trees
		void ToSubCube(TVec<int, Size> sc) //i - x 0 min 1 max    k - y min max   t - z min max
		{
			for (int i = 0; i < Size; i++)
			{
				if (sc[i] == 0)
					border[1][i] = (border[0][i] + border[1][i])*0.5f;
				else
					border[0][i] = (border[0][i] + border[1][i])*0.5f;
			}
		}

		BALULIB_DLL_INTERFACE virtual bool PointCollide(const TVec<T, Size>& point) const;
		BALULIB_DLL_INTERFACE virtual bool PointCollide(const TVec<T, Size>& point, TPointCollisionInfo<T, Size>& collision) const;
		BALULIB_DLL_INTERFACE virtual bool RayCollide(const TRay<T, Size> &ray) const;
		BALULIB_DLL_INTERFACE virtual bool RayCollide(const TRay<T, Size> &ray, TRayCollisionInfo<T, Size>& collision) const;
		BALULIB_DLL_INTERFACE virtual bool PlaneCollide(const TPlane<T, Size> &plane) const;
		BALULIB_DLL_INTERFACE virtual bool PlaneCollide(const TPlane<T, Size> &plane, TPlaneCollisionInfo<T, Size>& collision) const;
		BALULIB_DLL_INTERFACE virtual bool SegmentCollide(const TSegment<T, Size> &segment) const;
		BALULIB_DLL_INTERFACE virtual bool SegmentCollide(const TSegment<T, Size> &segment, TRayCollisionInfo<T, Size>& collision) const;
		BALULIB_DLL_INTERFACE virtual bool LineCollide(const TLine<T, Size> &line) const;
		BALULIB_DLL_INTERFACE virtual bool LineCollide(const TLine<T, Size> &line, TRayCollisionInfo<T, Size>& collision) const;

		BALULIB_DLL_INTERFACE virtual void DrawTriangles(std::vector<TVec<T, Size> >& vertices, std::vector<unsigned int>& indices)const;
		BALULIB_DLL_INTERFACE virtual void DrawLines(std::vector<TVec<T, Size> >& vertices)const;

		BALULIB_DLL_INTERFACE virtual bool CollideWith(const TBVolume<T, Size>& volume) const;
		BALULIB_DLL_INTERFACE virtual bool CollideWith(const TBVolume<T, Size>& volume, bool& fully_in_volume) const;
		BALULIB_DLL_INTERFACE virtual bool CollideWith(const TFrustum<T, Size>& frustum) const;
		BALULIB_DLL_INTERFACE virtual bool CollideWith(const TFrustum<T, Size>& frustum, bool& fully_in_frustum) const;
		BALULIB_DLL_INTERFACE virtual bool CollideWith(const TAABB<T, Size>& aabb) const;
		BALULIB_DLL_INTERFACE virtual bool CollideWith(const TAABB<T, Size>& aabb, bool& fully_in_aabb) const;
		BALULIB_DLL_INTERFACE virtual bool CollideWith(const TOBB<T, Size>& obb) const;
		BALULIB_DLL_INTERFACE virtual bool CollideWith(const TOBB<T, Size>& obb, bool& fully_in_obb) const;
		BALULIB_DLL_INTERFACE virtual bool CollideWith(const TCapsule<T, Size>& capsule) const;
		BALULIB_DLL_INTERFACE virtual bool CollideWith(const TCapsule<T, Size>& capsule, bool& fully_in_capsule) const;
		BALULIB_DLL_INTERFACE virtual bool CollideWith(const TSphere<T, Size>& sphere) const;
		BALULIB_DLL_INTERFACE virtual bool CollideWith(const TSphere<T, Size>& sphere, bool& fully_in_sphere) const;
	};

	typedef TAABB<float, 2>			TAABB2;
}
