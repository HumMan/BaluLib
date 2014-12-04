template<class T,int Size> 
bool Collide(const TCapsule<T,Size>& v0,const TCapsule<T,Size>& v1)
{
	return SegmentSegmentDistance(v0.segment, v1.segment) < v0.radius + v1.radius;
}