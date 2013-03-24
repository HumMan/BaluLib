
template<class T,int Size>
bool TOBB<T,Size>::Contain(const TVec<T,Size>& point) const
{
	return local.Contain(orient.TransMul(point-pos));
}

template<class T,int Size>
bool TOBB<T,Size>::Contain(const TVec<T,Size>& point,T& distance, TVec<T,Size>& nearest_point, TVec<T,Size>& normal) const
{
	bool result=local.Contain(orient.TransMul(point-pos),distance,nearest_point,normal);
	nearest_point=orient*nearest_point+pos;
	normal=orient*normal;
	return result;
}

template<class T,int Size>
bool TOBB<T,Size>::CollideWith(const TRay<T,Size> &ray) const
{
	return local.CollideWith(TRay<T,Size>(orient.TransMul(ray.pos-pos),orient.TransMul(ray.dir)));
}

template<class T,int Size>
bool TOBB<T,Size>::CollideWith(const TRay<T,Size> &ray, T& t, TVec<T,Size>& normal) const
{
	bool result = local.CollideWith(TRay<T,Size>(orient.TransMul(ray.pos-pos),orient.TransMul(ray.dir)),t,normal);
	normal=orient*normal;
	return result;
}

template<class T,int Size>
bool TOBB<T,Size>::CollideWith(const TRay<T,Size> &ray, T& t0,T& t1) const
{
	return local.CollideWith(TRay<T,Size>(orient.TransMul(ray.pos-pos),orient.TransMul(ray.dir)),t0,t1);
}

template<class T,int Size>
bool TOBB<T,Size>::CollideWith(const TRay<T,Size> &ray, T& t0, TVec<T,Size>& normal0,T& t1, TVec<T,Size>& normal1) const
{
	bool result = local.CollideWith(TRay<T,Size>(orient.TransMul(ray.pos-pos),orient.TransMul(ray.dir)),t0,normal0,t1,normal1);
	normal0=orient*normal0;
	normal1=orient*normal1;
	return result;
}