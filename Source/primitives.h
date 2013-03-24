#pragma once

//TODO организовать набор не объемных примитивов как BVolume (линии,отрезки,лучи,окружности,дуги), аможет это хрень

template<class T,int size>
struct TPlane
{
	TVec<T,size> normal;
	T dist;//расстояние от плоскости до начала координат в направлении нормали
	TPlane(){}

	TPlane(const TVec<T,4>& v)//TODO используется в Tfrustum непонятно что делает
	{
		COMPILE_TIME_ERR(size==3);
		T t=sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
		normal[0]=v[0]/t;
		normal[1]=v[1]/t;
		normal[2]=v[2]/t;
		dist=v[3]/t;
	}

	TPlane(const TVec<T,3>& v0,
		const TVec<T,3>& v1,
		const TVec<T,3>& v2)//плоскость по трем точкам
		:normal((v1-v0).Cross(v2-v0).GetNormalized()),dist(-(v0*normal))
	{
		COMPILE_TIME_ERR(size==3);
	}

	TPlane(const TVec<T,size>& use_normal,
		T use_dist)//по нормали и расстоянию
		:normal(use_normal),dist(use_dist){}

	TPlane(const TVec<T,size>& use_pos,
		const TVec<T,size>& use_normal)//по нормали и точке принадлежащей вершине
		:normal(use_normal),dist(-use_pos*use_normal){}

	T DistanceTo(const TVec<T,size>& v)const
	{
		return normal*v+dist;
	}
	TVec<T,size> Mirror(const TVec<T,size>& v)const//отражение точки относительно плоскости
	{
		return v+(normal*(v*normal+dist))*2;
	}
};

template<class T>
inline T To0_360Space(T angle)
{
	angle=fmod(angle,T(2*M_PI));
	if(angle<0)angle=2*M_PI+angle;
	return angle;
}

template<class T>
bool IsCCWMove(T a0,T a1,T& dist)
//result - направление наикратчайшего перемещения из a0 в a1
//dist - величина и знак перемещения (+ это CCW)
{
	a0=To0_360Space(a0);
	a1=To0_360Space(a1);
	T dist_0=abs(a0-a1);
	T dist_1=abs(2*M_PI-abs(dist_0));

	bool result=(dist_0<dist_1)?(a1>a0):(a1<a0);
	dist=(result?1.0:-1.0)*min(dist_0,dist_1);
	return result;
}

//TODO привести в нормальный вид, т.к. некторое функции уже имеются в bVolumes и сделать шаблонными и через тангенс
template<class T>
inline T AngleFromDir(const TVec<T,2>& v)
// v -  нормализованный вектор направления
// result - угол в радианах (-pi,pi)
{
	//assert(abs(v.Length()-1)<0.0000001);
	return (v[1]>=0?1:-1)*acos(v[0]);
}

template<class T,int Size>
inline T DistanceBetweenLinePoint(TVec<T,Size> v, TVec<T,Size> p0, TVec<T,Size> p1, T& t, TVec<T,Size>& nearest_point)
{
	TVec<T,Size> p01=p1-p0;
	t=(p01*(v-p0))/p01.SqrLength();
	t=Clamp<T>(0,1,t);
	nearest_point=p0+p01*t;
	return nearest_point.Distance(v);
}

template<class T,int Size>
inline T DistanceBetweenRayPoint(TVec<T,Size> v, TVec<T,Size> ray_pos, TVec<T,Size> ray_dir, T& t, TVec<T,Size>& nearest_point)
{
	t=ray_dir*(v-ray_pos);
	t=ClampMin<T>(0,t);
	nearest_point=ray_pos+ray_dir*t;
	return nearest_point.Distance(v);
}

template<class T>
inline bool CircleCircleCollide(TVec<T,2> center0,TVec<T,2> center1,T radius,TVec<T,2> p[])
{
	TVec<T,2> dir=center1-center0;
	T dist=dir.Length();
	if(dist>radius*2)return false;
	dir*=1.0/dist;
	dir=dir.Cross();
	TVec<T,2> middle=(center0+center1)*0.5;
	T t=sqrt(sqr(radius)-sqr(dist*0.5));
	p[0]=middle+dir*t;
	p[1]=middle-dir*t;
	return true;
}

template<class T>
inline int SegmentCircleCollide(TVec<T,2> s0,TVec<T,2> s1,TVec<T,2> center,T radius,TVec<T,2> p[])
{
	TVec<T,2> dir=s1-s0;
	TVec<T,2> b=s0-center;
	T discr=sqr(dir*b)-(dir*dir)*(b*b-sqr(radius));
	int result=0;
	if(discr>0)
	{
		T t0=(-dir*b-sqrt(discr))/(dir*dir);
		T t1=(-dir*b+sqrt(discr))/(dir*dir);
		if(t0>=0&&t0<=1.0)
		{
			p[result]=s0+dir*t0;
			result++;
		}
		if(t1>=0&&t1<=1.0)
		{
			p[result]=s0+dir*t1;
			result++;
		}
		return result;
	}else if(discr==0)
	{
		T t0=-(dir*b)/(dir*dir);
		if(t0>=0&&t0<=1.0)
		{
			p[result]=s0+dir*t0;
			result++;
		}
		return result;
	}else return 0;
}

template<class T>
inline int CircleCapsuleCollide(TVec<T,2> center,TVec<T,2> cap0,TVec<T,2> cap1,T radius,TVec<T,2> p[])
{
	TVec<T,2> dir=cap1-cap0;
	T dist=dir.Length();
	dir*=1.0/dist;
	TVec<T,2> temp[2];
	TVec<T,2> perp_dir=dir.Cross();
	int result=0,t;
	if((center-cap0)*dir<-radius)
		return CircleCircleCollide(center,cap0,radius,p)?2:0;
	else if((center-cap1)*dir>dist+radius)
		return CircleCircleCollide(center,cap1,radius,p)?2:0;
	else
	{	
		t=SegmentCircleCollide(cap0+perp_dir*radius,cap1+perp_dir*radius,center,radius,temp);
		for(int i=0;i<t;i++)
			p[result++]=temp[i];
		t=SegmentCircleCollide(cap0-perp_dir*radius,cap1-perp_dir*radius,center,radius,temp);
		for(int i=0;i<t;i++)
			p[result++]=temp[i];
		if(CircleCircleCollide(center,cap0,radius,temp))
			for(int i=0;i<2;i++)
				if((temp[i]-cap0)*dir<0)
					p[result++]=temp[i];
		if(CircleCircleCollide(center,cap1,radius,temp))
			for(int i=0;i<2;i++)
				if((temp[i]-cap1)*dir>0)
					p[result++]=temp[i];
		return result;
	}
}


//точки сферы задаются в порядке против часовой стрелки
template<class T>//TODO нифига не сфера для нее надо 4 точки это окружность
void CircleBy3Points(TVec<T,3> p0,TVec<T,3> p1,TVec<T,3> p2,TVec<T,3> &circle_pos)
{

	TVec<T,3> 
		normal=(p1-p0).Cross(p2-p0).GetNormalized(),
		p01=(p0+p1)*0.5,
		p02=(p0+p2)*0.5,
		c=p01-p02;
	TVec<T,3> 
		dir0=(p2-p0).GetRotated(normal,M_PI_2),
		dir1=(p1-p0).GetRotated(normal,M_PI_2);
	T det=dir0[0]*dir1[1]-dir1[0]*dir0[1];
	T t=(c[0]*dir1[1]-dir1[0]*c[1])/det;
	circle_pos=p02+dir0*t;
}

template<class T>//TODO нифига не сфера для нее надо 4 точки это окружность
void CircleBy3Points(TVec<T,2> p0,TVec<T,2> p1,TVec<T,2> p2,TVec<T,2> &circle_pos)
{

	TVec<T,2> 
		p01=(p0+p1)*0.5,
		p02=(p0+p2)*0.5,
		c=p01-p02;
	TVec<T,2> 
		dir0=(p2-p0).GetRotated(M_PI_2),
		dir1=(p1-p0).GetRotated(M_PI_2);
	T det=dir0[0]*dir1[1]-dir1[0]*dir0[1];
	T t=(c[0]*dir1[1]-dir1[0]*c[1])/det;
	circle_pos=p02+dir0*t;
}

//если в одной плоскости то ложь
template<class T>//TODO нифига не сфера для нее надо 4 точки это окружность
bool SphereBy4Points(TVec<T,3> p1,TVec<T,3> p2,TVec<T,3> p3,TVec<T,3> p4,TVec<T,3> &sphere_pos, T &radius)
{
	TMatrix<T,4> m11(
		p1[0],p1[1],p1[2],1,
		p2[0],p2[1],p2[2],1,
		p3[0],p3[1],p3[2],1,
		p4[0],p4[1],p4[2],1
		);
	TMatrix<T,4> m12(
		p1*p1,p1[1],p1[2],1,
		p2*p2,p2[1],p2[2],1,
		p3*p3,p3[1],p3[2],1,
		p4*p4,p4[1],p4[2],1
		);
	TMatrix<T,4> m13(
		p1*p1,p1[0],p1[2],1,
		p2*p2,p2[0],p2[2],1,
		p3*p3,p3[0],p3[2],1,
		p4*p4,p4[0],p4[2],1
		);
	TMatrix<T,4> m14(
		p1*p1,p1[0],p1[1],1,
		p2*p2,p2[0],p2[1],1,
		p3*p3,p3[0],p3[1],1,
		p4*p4,p4[0],p4[1],1
		);
	TMatrix<T,4> m15(
		p1*p1,p1[0],p1[1],p1[2],
		p2*p2,p2[0],p2[1],p2[2],
		p3*p3,p3[0],p3[1],p3[2],
		p4*p4,p4[0],p4[1],p4[2]
	);

	T m11_det=m11.GetDet();
	T m12_det=m12.GetDet();
	T m13_det=m13.GetDet();
	T m14_det=m14.GetDet();
	T m15_det=m15.GetDet();

	if(abs(m11_det)<0.00001)return false;
	sphere_pos[0]=0.5*m12_det/m11_det;
	sphere_pos[1]=-0.5*m13_det/m11_det;
	sphere_pos[2]=0.5*m14_det/m11_det;
	radius=sqrt(sphere_pos*sphere_pos-m15_det/m11_det);
	return true;
}

template<class T,int size>
struct TRay
{
	TVec<T,size> pos,dir;

	TRay(){}
	TRay(const TVec<T,size>& use_pos,const TVec<T,size>& use_dir)//по точке и направлению(должно быть нормализованным - чтобы работали все методы)
		:pos(use_pos),dir(use_dir)
	{
		//assert(IsIn((T)use_dir.Length(),(T)0.999,(T)1.001));
	}
	bool RayPlaneInters(const TRay& ray,const TPlane<T,size>& plane,TVec<T,size>& x)
	{
		//TODO  вроде проще так t = (plane.dist - Vector3.Dot(pos,plane.normal)) / Vector3.Dot(plane.normal,dir);
		float t=plane.normal*(plane.normal*plane.dist-ray.pos)/(plane.normal*ray.dir);
		if(t>0)
		{
			x=ray.pos+ray.dir*t;
			return true;
		}
		else return false;
	}
	TVec<T,size> RayInters(const TRay& ray)
	{
		COMPILE_TIME_ERR(size==2);//only for 2d rays
		T k=(pos[1]*dir[0]-ray.pos[1]*dir[0]+dir[1]*(ray.pos[0]-pos[0]))/
			(ray.dir[1]*dir[0]-ray.dir[0]*dir[1]);
		return ray.dir*k+ray.pos;
	}
	bool ConvexPolygonInters(const TVec<T,size>* v,int vertex_count,const TRay& ray,TVec<T,size>& x)
	{
		assert(vertex_count>2);
		TVec<T,size> normal=((v[1]-v[0]).Cross(v[2]-v[0])).GetNormalized();
		if(RayPlaneInters(ray,TPlane<T,size>(normal,normal*v[0]),x))
		{
			int i;
			for(i=0;i<vertex_count-1;i++)
			{
				if((x-v[i])*((v[i+1]-v[i]).Cross(normal))>0.0)return false;
			}
			if((x-v[i])*((v[0]-v[i]).Cross(normal))>0.0)return false;
			return true;
		}else return false;
	}
	T PointNearestToOtherRay(const TRay& otherRay)
	{
		TPlane<T,size> plane(otherRay.pos,((otherRay.dir.Cross(dir)).Cross(otherRay.dir)).GetNormalized());
		return plane.normal*
			(plane.normal*plane.dist-pos)/
			(plane.normal*dir);//TODO следует учесть что здесь не просто прямые, а лучи 
	}
	bool CylinderInters(
		TVec<T,size> cyl_pos,TVec<T,size> cyl_dir,
		double cyl_rad,double& lambda)
	{
		double d;
		double t,s;
		TVec<T,size> D,O;
		double in,out;
		TVec<T,size> RC=pos-cyl_pos;
		TVec<T,size> n=dir.Cross(cyl_dir);
		double ln=n.Length();
		n/=ln;
		d=fabs(RC*n);

		if (d<=cyl_rad)
		{
			O=RC.Cross(cyl_dir);
			t= - O*(n)/ln;
			O=n.Cross(cyl_dir);
			O.Normalize();
			s= fabs( sqrt(cyl_rad*cyl_rad - d*d) / (dir*O) );
			in=t-s;
			out=t+s;
			if (in<-0){
				if (out<-0) return 0;
				else lambda=out;
			}
			else
				if (out<-0) {
					lambda=in;
				}
				else
					if (in<out) lambda=in;
					else lambda=out;
					return 1;
		}
		return 0;
	}
	bool PlaneInters(const TPlane<T,size>& plane,TVec<T,size>& x)
	{
		T t;
		if(RayPlaneInters(plane,t))
		{
			x=pos+dir*t;
			return true;
		}
		return false;
	}
	bool PlaneInters(const TPlane<T,size>& plane,T& t)
	{
		//TODO  вроде проще так t = (plane.dist - Vector3.Dot(pos,plane.normal)) / Vector3.Dot(plane.normal,dir);
		t=plane.normal*(plane.normal*plane.dist-pos)/(plane.normal*dir);
		return t>0;
	}
};