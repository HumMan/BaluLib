#pragma once

template<class T, int Size>
bool Collide(const TSphere<T,Size> &v0,const TSphere<T,Size> &v1)
{
	return v0.pos.SqrDistance(v1.pos)<sqr(v0.radius+v1.radius);
}

template<class T,int Size>
bool Collide(const TSphere<T,Size> &v0,const TSphere<T,Size> &v1,bool& v1_fullin_v0)
{
	T sqr_dist=v0.pos.SqrDistance(v1.pos);
        v1_fullin_v0=sqr_dist<sqr(v0.radius-v1.radius);
	return sqr_dist<sqr(v0.radius+v1.radius);
}

template<class T,int Size>
bool Collide(const TSphere<T,Size> &v0,const TSphere<T,Size> &v1,T& sqr_distance)
{
        T sqr_dist=v0.pos.SqrDistance(v1.pos);
	return sqr_dist<sqr(v0.radius+v1.radius);
}
