
template<class T,int Size>
bool TCapsule<T,Size>::Contain(const TVec<T,Size>& point) const
{
	T t;
	TVec<T,Size> nearest_point;
	return DistanceBetweenLinePoint<T,Size>(point,p0,p1,t,nearest_point)<radius;
}

template<class T,int Size>
bool TCapsule<T,Size>::Contain(const TVec<T,Size>& point,T& distance, TVec<T,Size>& nearest_point, TVec<T,Size>& normal) const
{
	T t;
	distance=DistanceBetweenLinePoint<T,Size>(point,p0,p1,t,nearest_point);
	bool result=distance<radius;
	normal=(point-nearest_point)*(1/distance);
	return result;
}

template<class T,int Size>
bool TCapsule<T,Size>::CollideWith(const TRay<T,Size> &ray) const
{
	if(false)
	{
		//TODO частично работает
		//проверяется пересечение по SAT
		TVec<T,Size> _p0=p0-ray.pos;
		TVec<T,Size> _p1=p1-ray.pos;
		TVec<T,Size> middle=(_p0+_p1)*0.5;//middle of capsule
		TVec<T,Size> size=_p1-middle;//half vector of capsule segment
		TVec<T,Size> caps_dir=size.GetNormalized();

		//проекции на ось луча
		if(ray.dir*middle+radius+abs(size*ray.dir)<0)
			return false;

		//проекции на ось капсулы
		T p=-(caps_dir*middle);
		if(caps_dir*ray.dir>0){
			if(p>size.Length()+radius)return false;
		}else if(p<-size.Length()-radius)return false;

		//проекции на cross(ось луча, ось капсулы)
		TVec<T,Size> cross_dir=ray.dir.Cross(caps_dir).GetNormalized();
		if(abs(cross_dir*middle)>radius/*+size.AbsScalarMul(cross_dir)*/)return false;

		//
		TVec<T,Size> n0;
		T t;

		//DistanceBetweenLinePoint(ray.pos,middle-size,middle+size,t,n0);
		//if(t>0.00001&&t<1.0-0.00001)
		//{
		//}

		T l=DistanceBetweenRayPoint(_p0,TVec<T,Size>(0),ray.dir,t,n0);
		if(t>0.00001)
		{
			T temp=(size*2)*((n0-_p0).GetNormalized());
			if(temp<0)
			{
				if(radius<l)return false;
			}else
				if(radius+temp<l)return false;
		}

		l=DistanceBetweenRayPoint(_p1,TVec<T,Size>(0),ray.dir,t,n0);
		if(t>0.00001)
		{
			T temp=(size*(-2))*((n0-_p1).GetNormalized());
			if(temp<0)
			{
				if(radius<l)return false;
			}else
				if(radius+temp<l)return false;
		}

		//v=middle+size;
		//DistanceBetweenRayPoint(v,ray.pos,ray.dir,t,n0);
		//if(t>0.00001)
		//{
		//	if(abs((n0-v).GetNormalized()*(-size*2))+radius<(n0-v).Length())
		//		return false;
		//}

		return true;
	}else
	{
		//TODO сделать именно расстояние между лучом и отрезком а не 2мя отрезками
		//http://www.softsurfer.com/Archive/algorithm_0106/algorithm_0106.htm#dist3D_Segment_to_Segment()
		const T SMALL_NUM=0.000001;
		TVec<T,Size>   u = p1 - p0;
		TVec<T,Size>   v = /*ray.pos+*/ray.dir*10000/* - ray.pos*/;
		TVec<T,Size>   w = p0 - ray.pos;
		T    a = (u*u);        // always >= 0
		T    b = (u*v);
		T    c = (v*v);        // always >= 0
		T    d = (u*w);
		T    e = (v*w);
		T    D = a*c - b*b;       // always >= 0
		T    sc, sN, sD = D;      // sc = sN / sD, default sD = D >= 0
		T    tc, tN, tD = D;      // tc = tN / tD, default tD = D >= 0

		// compute the line parameters of the two closest points
		if (D < SMALL_NUM) { // the lines are almost parallel
			sN = 0.0;        // force using point P0 on segment S1
			sD = 1.0;        // to prevent possible division by 0.0 later
			tN = e;
			tD = c;
		}
		else {                // get the closest points on the infinite lines
			sN = (b*e - c*d);
			tN = (a*e - b*d);
			if (sN < 0.0) {       // sc < 0 => the s=0 edge is visible
				sN = 0.0;
				tN = e;
				tD = c;
			}
			else if (sN > sD) {  // sc > 1 => the s=1 edge is visible
				sN = sD;
				tN = e + b;
				tD = c;
			}
		}

		if (tN < 0.0) {           // tc < 0 => the t=0 edge is visible
			tN = 0.0;
			// recompute sc for this edge
			if (-d < 0.0)
				sN = 0.0;
			else if (-d > a)
				sN = sD;
			else {
				sN = -d;
				sD = a;
			}
		}
		else if (tN > tD) {      // tc > 1 => the t=1 edge is visible
			tN = tD;
			// recompute sc for this edge
			if ((-d + b) < 0.0)
				sN = 0;
			else if ((-d + b) > a)
				sN = sD;
			else {
				sN = (-d + b);
				sD = a;
			}
		}
		// finally do the division to get sc and tc
		sc = (abs(sN) < SMALL_NUM ? 0.0 : sN / sD);
		tc = (abs(tN) < SMALL_NUM ? 0.0 : tN / tD);

		// get the difference of the two closest points
		TVec<T,Size>   dP = w + (u*sc) - (v*tc);  // = S1(sc) - S2(tc)

		return dP.Length()<radius;   // return the closest distance
	}
}

template<class T,int Size>
bool TCapsule<T,Size>::CollideWith(const TRay<T,Size> &ray, T& t, TVec<T,Size>& normal) const
{
	return false;
}

template<class T,int Size>
bool TCapsule<T,Size>::CollideWith(const TRay<T,Size> &ray, T& t0,T& t1)const
{
	return false;
}

template<class T,int Size>
bool TCapsule<T,Size>::CollideWith(const TRay<T,Size> &ray, T& t0, TVec<T,Size>& normal0,T& t1, TVec<T,Size>& normal1) const
{
	return false;
}