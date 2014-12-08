#pragma once

template<class T, int Size>
bool Collide(const TCapsule<T,Size>& v0,const TOBB<T,Size>& v1)
{
	TCapsule<T, Size> transformed(v1.orient.TransMul(v0.segment.p0 - v1.pos), v1.orient.TransMul(v0.segment.p1 - v1.pos), v0.radius);
	return v1.local.CollideWith(transformed);
}

template<class T, int Size>
bool Collide(const TCapsule<T, Size>& v0, const TOBB<T, Size>& v1, bool& fully_in_obb)
{
	TCapsule<T, Size> transformed(v1.orient.TransMul(v0.segment.p0 - v1.pos), v1.orient.TransMul(v0.segment.p1 - v1.pos), v0.radius);
	return Collide(transformed, v1.local, fully_in_obb);
}

template<class T, int Size>
bool Collide(const TOBB<T, Size>& v1, const TCapsule<T, Size>& v0, bool& fully_in_capsule)
{
	TCapsule<T, Size> transformed(v1.orient.TransMul(v0.segment.p0 - v1.pos), v1.orient.TransMul(v0.segment.p1 - v1.pos), v0.radius);
	return v1.local.CollideWith(transformed, fully_in_capsule);
}