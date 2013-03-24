template<class T,int Size>
bool TSphere<T,Size>::Contain(const TVec<T,Size>& point) const
{
	return point.SqrDistance(pos)<radius*radius;
}

template<class T,int Size>
bool TSphere<T,Size>::Contain(const TVec<T,Size>& point,T& distance, TVec<T,Size>& nearest_point, TVec<T,Size>& normal) const
{
	TVec<T,Size> temp=point-pos;
	normal=temp.GetNormalized();
	nearest_point=normal*radius;
	return temp.SqrLength()<radius*radius;
}

template<class T,int Size>
bool TSphere<T,Size>::CollideWith(const TRay<T,Size> &ray) const
{
	if(false)
	{
		T t0,t1;
		return CollideWith(ray,t0,t1);
	}else
	{
		//
		TVec<T,Size> diff = ray.pos - pos;
		T a0 = diff*diff -sqr(radius);
		if (a0 <= 0)
		{
			// P is inside the sphere
			return true;
		}
		// else: P is outside the sphere

		T a1 = ray.dir*diff;
		if (a1 >= 0)
		{
			return false;
		}

		// Quadratic has a real root if discriminant is nonnegative.
		return a1*a1 >= a0;
	}
}

template<class T,int Size>
bool TSphere<T,Size>::CollideWith(const TRay<T,Size> &ray, T& t, TVec<T,Size>& normal) const
{
	T t0,t1;
	if(CollideWith(ray,t0,t1))
	{
		T t=t0>0?t0:t1;
		normal=(ray.pos+ray.dir*t-pos).GetNormalized();
		return true;
	}else return false;
}

template<class T,int Size>
bool TSphere<T,Size>::CollideWith(const TRay<T,Size> &ray, T& t0,T& t1) const
{
	T a,b,c;
	TVec<T,Size> t,d;
	d=ray.pos-pos;
	a=(ray.dir*ray.dir);
	b=(ray.dir*d)*2.0f;
	c=(d*d-radius*radius);
	T discr=b*b-4*a*c;
	if(discr<0)return false;
	discr=sqrt(discr);
	t0=(b+discr)/(-2*a);
	t1=(b-discr)/(-2*a);
	assert(t1>=t0);
	if(t0<0&&t1<0)return false;
	return true;
}
template<class T,int Size>
bool TSphere<T,Size>::CollideWith(const TRay<T,Size> &ray, T& t0, TVec<T,Size>& normal0,T& t1, TVec<T,Size>& normal1) const
{
	if(CollideWith(ray,t0,t1))
	{
		normal0=(ray.pos+ray.dir*t0-pos).GetNormalized();
		normal1=(ray.pos+ray.dir*t1-pos).GetNormalized();
		return true;
	}else return false;
}