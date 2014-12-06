#pragma once

#include "AABBAndSphere.h"

template<class T,int Size>
bool Collide(const TSphere<T,Size>& v0,const TOBB<T,Size>& v1)
{
	TVec<T,Size> local_pos=v1.orient.TransMul(v0.pos-v1.pos);
	return Collide(v1.local,TSphere<T,Size>(local_pos,v0.radius));
}

template<class T, int Size>
bool Collide(const TSphere<T, Size>& v0, const TOBB<T, Size>& v1, bool& fully_in_obb)
{
	return false; //TODO
}

template<class T, int Size>
bool Collide(const TOBB<T, Size>& v1, const TSphere<T, Size>& v0, bool& fully_in_sphere)
{
	return false; //TODO
}
