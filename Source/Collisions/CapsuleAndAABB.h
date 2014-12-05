#pragma once

template<class T, int Size>
bool Collide(const TCapsule<T,Size>& capsule,const TAABB<T,Size>& aabb)
{
	TRayCollisionInfo<T, Size> collision;
	aabb.SegmentCollide(capsule.segment, collision);

}

template<class T, int Size>
bool Collide(const TCapsule<T, Size>& capsule, const TAABB<T, Size>& aabb, bool& capsule_fully_in_aabb)
{
	return false;//TODO
}

template<class T, int Size>
bool Collide(const TAABB<T, Size>& aabb, const TCapsule<T, Size>& capsule, bool& aabb_fully_in_capsule)
{
	return false;//TODO
}

