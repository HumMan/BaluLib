template<class T,int Size>
bool Collide(const TOBB<T,Size>& v0,const TOBB<T,Size>& v1)
{
	TOBB<T,Size> transformed(v0.orient.TransMul(v1.pos-v0.pos),v0.orient*(v1.orient.GetTransposed()),v1.local);
	return v0.local.CollideWith(transformed);
}