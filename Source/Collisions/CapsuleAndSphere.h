template<class T,int Size> 
bool Collide(const TCapsule<T,Size>& v0,const TSphere<T,Size>& v1)
{
	TVec<T, Size> nearest_point;
	T t;
	return DistanceBetweenPointLine(v1.pos, v0.segment.p0, v0.segment.p1, t, nearest_point) < v0.radius + v1.radius;
}