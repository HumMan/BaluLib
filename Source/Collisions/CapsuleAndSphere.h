#pragma once

template<class T, int Size>
bool Collide(const TCapsule<T, Size>& capsule, const TSphere<T, Size>& sphere)
{
	TVec<T, Size> nearest_point;
	T t;
	return DistanceBetweenPointSegment(sphere.pos, capsule.segment, t, nearest_point) < capsule.radius + sphere.radius;
}

template<class T, int Size>
bool Collide(const TCapsule<T, Size>& capsule, const TSphere<T, Size>& sphere, bool& capsule_fully_in_sphere)
{
	TVec<T, Size> nearest_point;
	T t;
	
	capsule_fully_in_sphere =
		capsule.segment.p0.SqrDistance(sphere.pos) < min_r &&
		capsule.segment.p1.SqrDistance(sphere.pos) < min_r;
	return DistanceBetweenPointSegment(sphere.pos, capsule.segment, t, nearest_point) < capsule.radius + sphere.radius;
}

template<class T, int Size>
bool Collide(const TCapsule<T, Size>& sphere, const TSphere<T, Size>& capsule, bool& sphere_fully_in_capsule)
{
	TVec<T, Size> nearest_point;
	T t;
	T dist = DistanceBetweenPointSegment(sphere.pos, capsule.segment, t, nearest_point);
	T min_r = (capsule.radius - sphere.radius);
	sphere_fully_in_capsule = dist < min_r;
	return dist < capsule.radius + sphere.radius;
}
