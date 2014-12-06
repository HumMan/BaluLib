#pragma once

template<class T, int Size>
bool Collide(const TCapsule<T,Size>& v0,const TCapsule<T,Size>& v1)
{
	return SegmentSegmentDistance(v0.segment, v1.segment) < v0.radius + v1.radius;
}

template<class T, int Size>
bool Collide(const TCapsule<T, Size>& v0, const TCapsule<T, Size>& v1, bool& capsule0_fully_in_capsule1)
{
	T t;
	TVec<T, Size> nearest_point;
	capsule0_fully_in_capsule1 =
		DistanceBetweenPointSegment(v0.segment.p0, v1.segment, t, nearest_point ) < v0.radius + v1.radius &&
		DistanceBetweenPointSegment(v0.segment.p1, v1.segment, t, nearest_point) < v0.radius + v1.radius;
	return SegmentSegmentDistance(v0.segment, v1.segment) < v0.radius + v1.radius;
}