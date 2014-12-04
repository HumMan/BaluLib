#include "../../BVolumes/Capsule.h"

template<class T,int Size>
bool TCapsule<T,Size>::Contain(const TVec<T,Size>& point) const
{
	T t;
	TVec<T,Size> nearest_point;
	return DistanceBetweenPointLine<T, Size>(point, segment.p0, segment.p1, t, nearest_point)<radius;
}

template<class T,int Size>
bool TCapsule<T,Size>::Contain(const TVec<T,Size>& point,T& distance, TVec<T,Size>& nearest_point, TVec<T,Size>& normal) const
{
	T t;
	distance = DistanceBetweenPointLine<T, Size>(point, segment.p0, segment.p1, t, nearest_point);
	bool result=distance<radius;
	normal=(point-nearest_point)*(1/distance);
	return result;
}


template<class T,int Size>
bool TCapsule<T,Size>::CollideWith(const TRay<T,Size> &ray) const
{
	T ray_t, capsule_t;
	return SegmentRayDistance(segment, ray, capsule_t, ray_t)<radius;
}

template<class T,int Size>
bool TCapsule<T,Size>::CollideWith(const TRay<T,Size> &ray, T& t, TVec<T,Size>& normal) const
{
	return false;
}

template<class T,int Size>
bool TCapsule<T,Size>::CollideWith(const TRay<T,Size> &ray, T& t0,T& t1)const
{
	return false;
}

template<class T,int Size>
bool TCapsule<T,Size>::CollideWith(const TRay<T,Size> &ray, T& t0, TVec<T,Size>& normal0,T& t1, TVec<T,Size>& normal1) const
{
	T ray_t, capsule_t;
	T dist = SegmentRayDistance(segment, ray, capsule_t, ray_t);
	//TVec<T, Size> ray_pos = ray.pos + ray.dir*ray_t;
	//TVec<T, Size> seg_pos = segment.p0 + (segment.p1 - segment.p0).GetNormalized()*capsule_t;
	//normal0 = (ray_pos - seg_pos).GetNormalized();
	return dist<radius;
}

template class TCapsule<float, 2>;
template class TCapsule<float, 3>;
template class TCapsule<double, 2>;
template class TCapsule<double, 3>;