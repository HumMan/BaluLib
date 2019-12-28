#pragma once

#include <vector>

#include "../Source/Math/vec.h"
#include "../Source/Math/matrix.h"
#include "../Source/stdfuncs.h"

namespace BaluLib
{

	template<class T, int size>
	struct TRay;

	template<class T, int size>
	struct TPlane
	{
		//эквивалентно уравнению плоскости ax + by + cz + d = 0 т.е. normal*v + dist = 0
		TVec<T, size> normal;
		T dist;

		TPlane(){}

		TPlane(const TVec<T, 4>& v)//используетс¤ в TFrustum - создание плоскости из не нормализованного уравнени¤ ax+by+cz+d=0, где вектор v соответствует (a,b,c,d)
		{
			static_assert(size == 3, "supports only 3d");
			//вектор нормали плоскости (a,b,c) необходимо нормализовать
			T t = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
			normal[0] = v[0] / t;
			normal[1] = v[1] / t;
			normal[2] = v[2] / t;
			dist = v[3] / t;
		}

		TPlane(const TVec<T, 3>& v0,
			const TVec<T, 3>& v1,
			const TVec<T, 3>& v2)//плоскость по трем точкам
			:normal((v1 - v0).Cross(v2 - v0).GetNormalized()), dist(-(v0*normal))
		{
			static_assert(size == 3, "supports only 3d");
		}

		TPlane(const TVec<T, size>& use_pos,
			const TVec<T, size>& use_normal)//по нормали и точке принадлежащей плоскости
			:normal(use_normal), dist(-use_pos*use_normal){}

		T DistanceTo(const TVec<T, size>& v)const // T>0 если pos со стороны нормали плоскости
		{
			return normal*v + dist;
		}
		TVec<T, size> Mirror(const TVec<T, size>& v)const//отражение точки относительно плоскости
		{
			return v + (normal*(v*normal + dist)) * 2;
		}
		TVec<T, size> GetPos()const
		{
			return normal*(-dist);
		}
	};


	template<class T, int size>
	struct TSegment
	{
		TVec<T, size> p0, p1;
		TSegment()
		{
		}
		TSegment(const TVec<T, size>& p0, const TVec<T, size>& p1)
		{
			this->p0 = p0;
			this->p1 = p1;
		}
		TVec<T, size> GetDir()const
		{
			return (p1 - p0).GetNormalized();
		}
		TRay<T, size> ToRay()const
		{
			return TRay<T, size>(p0, GetDir());
		}
	};

	template<class T, int size>
	struct TLine
	{
		TVec<T, size> p0, dir;

		TLine()
		{
		}
		TLine(const TVec<T, size>& p0, const TVec<T, size>& p1)
		{
			this->p0 = p0;
			this->dir = (p1 - p0).GetNormalized();
		}
		TRay<T, size> ToRay()const
		{
			return TRay<T, size>(p0, dir);
		}
	};

	template<class T>
	inline T To0_360Space(T angle)
	{
		angle = fmod(angle, T(2 * M_PI));
		if (angle < 0)angle = 2 * M_PI + angle;
		return angle;
	}

	template<class T>
	bool IsCCWMove(T a0, T a1, T& dist)
		//result - направление наикратчайшего перемещени¤ из a0 в a1
		//dist - величина и знак перемещени¤ (+ это CCW)
	{
		a0 = To0_360Space(a0);
		a1 = To0_360Space(a1);
		T dist_0 = std::abs(a0 - a1);
		T dist_1 = std::abs(2 * M_PI - std::abs(dist_0));

		bool result = (dist_0<dist_1) ? (a1>a0) : (a1 < a0);
		dist = (result ? 1.0 : -1.0)*min(dist_0, dist_1);
		return result;
	}

	//TODO привести в нормальный вид, т.к. некторое функции уже имеютс¤ в bVolumes и сделать шаблонными и через тангенс
	template<class T>
	inline T AngleFromDir(const TVec<T, 2>& v)
		// v -  нормализованный вектор направлени¤
		// result - угол в радианах (-pi,pi)
	{
		//assert(std::abs(v.Length()-1)<0.0000001);
		return (v[1] >= 0 ? 1 : -1)*acos(v[0]);
	}

	template<class T, int Size>
	inline T DistanceBetweenPointLine(TVec<T, Size> point, TLine<T, Size> line)
	{
		TVec<T, Size> ap = line.p0 - point;
		T proj_on_line = ap*line.dir;
		TVec<T, Size> proj_on_line_vector = line.dir * proj_on_line;
		TVec<T, Size> dist_vector = ap - proj_on_line_vector;
		return dist_vector.Length();
	}

	///t - от 0 до 1 (где 0 это p0, 1 это p1)
	template<class T, int Size>
	inline T DistanceBetweenPointSegment(TVec<T, Size> point, TSegment<T, Size> segment, T& t, TVec<T, Size>& nearest_point)
	{
		TVec<T, Size> p01 = segment.p1 - segment.p0;
		t = (p01*(point - segment.p0)) / p01.SqrLength();
		t = Clamp<T>(0, 1, t);
		nearest_point = segment.p0 + p01*t;
		return nearest_point.Distance(point);
	}

	template<class T, int Size>
	inline T DistanceBetweenPointSegment(TVec<T, Size> point, TSegment<T, Size> segment)
	{
		TVec<T, Size> p01 = segment.p1 - segment.p0;
		T t = (p01*(point - segment.p0)) / p01.SqrLength();
		t = Clamp<T>(0, 1, t);
		TVec<T, Size>  nearest_point = segment.p0 + p01*t;
		return nearest_point.Distance(point);
	}

	template<class T, int Size>
	inline T DistanceBetweenRayPoint(TVec<T, Size> v, TVec<T, Size> ray_pos, TVec<T, Size> ray_dir, T& t, TVec<T, Size>& nearest_point)
	{
		t = ray_dir*(v - ray_pos);
		t = ClampMin<T>(0, t);
		nearest_point = ray_pos + ray_dir*t;
		return nearest_point.Distance(v);
	}

	template<class T>
	inline bool CircleCircleCollide(TVec<T, 2> center0, TVec<T, 2> center1, T radius, TVec<T, 2> p[])
	{
		TVec<T, 2> dir = center1 - center0;
		T dist = dir.Length();
		if (dist > radius * 2)return false;
		dir *= 1.0 / dist;
		dir = dir.Cross();
		TVec<T, 2> middle = (center0 + center1)*0.5;
		T t = sqrt(sqr(radius) - sqr(dist*0.5));
		p[0] = middle + dir*t;
		p[1] = middle - dir*t;
		return true;
	}

	template<class T>
	inline int SegmentCircleCollide(TVec<T, 2> s0, TVec<T, 2> s1, TVec<T, 2> center, T radius, TVec<T, 2> p[])
	{
		TVec<T, 2> dir = s1 - s0;
		TVec<T, 2> b = s0 - center;
		T discr = sqr(dir*b) - (dir*dir)*(b*b - sqr(radius));
		int result = 0;
		if (discr > 0)
		{
			T t0 = (-dir*b - sqrt(discr)) / (dir*dir);
			T t1 = (-dir*b + sqrt(discr)) / (dir*dir);
			if (t0 >= 0 && t0 <= 1.0)
			{
				p[result] = s0 + dir*t0;
				result++;
			}
			if (t1 >= 0 && t1 <= 1.0)
			{
				p[result] = s0 + dir*t1;
				result++;
			}
			return result;
		}
		else if (discr == 0)
		{
			T t0 = -(dir*b) / (dir*dir);
			if (t0 >= 0 && t0 <= 1.0)
			{
				p[result] = s0 + dir*t0;
				result++;
			}
			return result;
		}
		else return 0;
	}

	template<class T>
	inline int CircleCapsuleCollide(TVec<T, 2> center, TVec<T, 2> cap0, TVec<T, 2> cap1, T radius, TVec<T, 2> p[])
	{
		TVec<T, 2> dir = cap1 - cap0;
		T dist = dir.Length();
		dir *= 1.0 / dist;
		TVec<T, 2> temp[2];
		TVec<T, 2> perp_dir = dir.Cross();
		int result = 0, t;
		if ((center - cap0)*dir<-radius)
			return CircleCircleCollide(center, cap0, radius, p) ? 2 : 0;
		else if ((center - cap1)*dir>dist + radius)
			return CircleCircleCollide(center, cap1, radius, p) ? 2 : 0;
		else
		{
			t = SegmentCircleCollide(cap0 + perp_dir*radius, cap1 + perp_dir*radius, center, radius, temp);
			for (int i = 0; i < t; i++)
				p[result++] = temp[i];
			t = SegmentCircleCollide(cap0 - perp_dir*radius, cap1 - perp_dir*radius, center, radius, temp);
			for (int i = 0; i < t; i++)
				p[result++] = temp[i];
			if (CircleCircleCollide(center, cap0, radius, temp))
				for (int i = 0; i < 2; i++)
					if ((temp[i] - cap0)*dir < 0)
						p[result++] = temp[i];
			if (CircleCircleCollide(center, cap1, radius, temp))
				for (int i = 0; i < 2; i++)
					if ((temp[i] - cap1)*dir>0)
						p[result++] = temp[i];
			return result;
		}
	}


	//точки сферы задаютс¤ в пор¤дке против часовой стрелки
	template<class T>//TODO нифига не сфера дл¤ нее надо 4 точки это окружность
	void CircleBy3Points(TVec<T, 3> p0, TVec<T, 3> p1, TVec<T, 3> p2, TVec<T, 3> &circle_pos)
	{

		TVec<T, 3>
			normal = (p1 - p0).Cross(p2 - p0).GetNormalized(),
			p01 = (p0 + p1)*0.5,
			p02 = (p0 + p2)*0.5,
			c = p01 - p02;
		TVec<T, 3>
			dir0 = (p2 - p0).GetRotated(normal, M_PI_2),
			dir1 = (p1 - p0).GetRotated(normal, M_PI_2);
		T det = dir0[0] * dir1[1] - dir1[0] * dir0[1];
		T t = (c[0] * dir1[1] - dir1[0] * c[1]) / det;
		circle_pos = p02 + dir0*t;
	}

	template<class T>//TODO нифига не сфера дл¤ нее надо 4 точки это окружность
	void CircleBy3Points(TVec<T, 2> p0, TVec<T, 2> p1, TVec<T, 2> p2, TVec<T, 2> &circle_pos)
	{

		TVec<T, 2>
			p01 = (p0 + p1)*0.5,
			p02 = (p0 + p2)*0.5,
			c = p01 - p02;
		TVec<T, 2>
			dir0 = (p2 - p0).GetRotated(M_PI_2),
			dir1 = (p1 - p0).GetRotated(M_PI_2);
		T det = dir0[0] * dir1[1] - dir1[0] * dir0[1];
		T t = (c[0] * dir1[1] - dir1[0] * c[1]) / det;
		circle_pos = p02 + dir0*t;
	}

	//если в одной плоскости то ложь
	template<class T>//TODO нифига не сфера дл¤ нее надо 4 точки это окружность
	bool SphereBy4Points(TVec<T, 3> p1, TVec<T, 3> p2, TVec<T, 3> p3, TVec<T, 3> p4, TVec<T, 3> &sphere_pos, T &radius)
	{
		TMatrix<T, 4> m11(
			p1[0], p1[1], p1[2], 1,
			p2[0], p2[1], p2[2], 1,
			p3[0], p3[1], p3[2], 1,
			p4[0], p4[1], p4[2], 1
			);
		TMatrix<T, 4> m12(
			p1*p1, p1[1], p1[2], 1,
			p2*p2, p2[1], p2[2], 1,
			p3*p3, p3[1], p3[2], 1,
			p4*p4, p4[1], p4[2], 1
			);
		TMatrix<T, 4> m13(
			p1*p1, p1[0], p1[2], 1,
			p2*p2, p2[0], p2[2], 1,
			p3*p3, p3[0], p3[2], 1,
			p4*p4, p4[0], p4[2], 1
			);
		TMatrix<T, 4> m14(
			p1*p1, p1[0], p1[1], 1,
			p2*p2, p2[0], p2[1], 1,
			p3*p3, p3[0], p3[1], 1,
			p4*p4, p4[0], p4[1], 1
			);
		TMatrix<T, 4> m15(
			p1*p1, p1[0], p1[1], p1[2],
			p2*p2, p2[0], p2[1], p2[2],
			p3*p3, p3[0], p3[1], p3[2],
			p4*p4, p4[0], p4[1], p4[2]
			);

		T m11_det = m11.GetDet();
		T m12_det = m12.GetDet();
		T m13_det = m13.GetDet();
		T m14_det = m14.GetDet();
		T m15_det = m15.GetDet();

		if (std::abs(m11_det) < 0.00001)return false;
		sphere_pos[0] = 0.5*m12_det / m11_det;
		sphere_pos[1] = -0.5*m13_det / m11_det;
		sphere_pos[2] = 0.5*m14_det / m11_det;
		radius = sqrt(sphere_pos*sphere_pos - m15_det / m11_det);
		return true;
	}

	template<class T, int size>
	struct TRay
	{
		TVec<T, size> pos, dir;

		TRay(){}
		TRay(const TVec<T, size>& use_pos, const TVec<T, size>& use_dir)//по точке и направлению(должно быть нормализованным - чтобы работали все методы)
			:pos(use_pos), dir(use_dir)
		{
			assert(std::abs(use_dir.Length() - 1) < (T)0.00001);
		}
		bool RayPlaneInters(const TRay& ray, const TPlane<T, size>& plane, TVec<T, size>& x)
		{
			//TODO  вроде проще так t = (plane.dist - Vector3.Dot(pos,plane.normal)) / Vector3.Dot(plane.normal,dir);
			float t = plane.normal*(plane.normal*plane.dist - ray.pos) / (plane.normal*ray.dir);
			if (t > 0)
			{
				x = ray.pos + ray.dir*t;
				return true;
			}
			else return false;
		}
		TVec<T, size> RayInters(const TRay& ray)
		{
			assert(size == 2);//only for 2d rays
			T k = (pos[1] * dir[0] - ray.pos[1] * dir[0] + dir[1] * (ray.pos[0] - pos[0])) /
				(ray.dir[1] * dir[0] - ray.dir[0] * dir[1]);
			return ray.dir*k + ray.pos;
		}
		bool ConvexPolygonInters(const TVec<T, size>* v, int vertex_count, const TRay& ray, TVec<T, size>& x)
		{
			assert(vertex_count > 2);
			TVec<T, size> normal = ((v[1] - v[0]).Cross(v[2] - v[0])).GetNormalized();
			if (RayPlaneInters(ray, TPlane<T, size>(normal, normal*v[0]), x))
			{
				int i;
				for (i = 0; i<vertex_count - 1; i++)
				{
					if ((x - v[i])*((v[i + 1] - v[i]).Cross(normal))>0.0)return false;
				}
				if ((x - v[i])*((v[0] - v[i]).Cross(normal)) > 0.0)return false;
				return true;
			}
			else return false;
		}
		T PointNearestToOtherRay(const TRay& otherRay)
		{
			TPlane<T, size> plane(otherRay.pos, ((otherRay.dir.Cross(dir)).Cross(otherRay.dir)).GetNormalized());
			return plane.normal*
				(plane.normal*plane.dist - pos) /
				(plane.normal*dir);//TODO следует учесть что здесь не просто пр¤мые, а лучи
		}

	};

	template<class T, int size>
	bool RayCylinderCollide(const TRay<T, size> ray, TVec<T, size> cyl_pos, TVec<T, size> cyl_dir, double cyl_rad, double& lambda)
	{
		double d;
		double t, s;
		TVec<T, size> D, O;
		double in, out;
		TVec<T, size> RC = ray.pos - cyl_pos;
		TVec<T, size> n = ray.dir.Cross(cyl_dir);
		double ln = n.Length();
		n /= ln;
		d = std::fabs(RC*n);

		if (d <= cyl_rad)
		{
			O = RC.Cross(cyl_dir);
			t = -O*(n) / ln;
			O = n.Cross(cyl_dir);
			O.Normalize();
			s = std::fabs(sqrt(cyl_rad*cyl_rad - d*d) / (ray.dir*O));
			in = t - s;
			out = t + s;
			if (in < -0){
				if (out < -0) return 0;
				else lambda = out;
			}
			else
				if (out < -0) {
					lambda = in;
				}
				else {
					if (in < out) lambda = in;
					else lambda = out;
					return 1;
				}
		}
		return 0;
	}

	template<class T, int size>
	bool RayPlaneCollide(const TRay<T, size> ray, const TPlane<T, size>& plane, TVec<T, size>& x)
	{
		T t;
		if (RayPlaneCollide(plane, t))
		{
			x = ray.pos + ray.dir*t;
			return true;
		}
		return false;
	}

	template<class T, int size>
	bool RayPlaneCollide(const TRay<T, size> ray, const TPlane<T, size>& plane, T& t)
	{
		//TODO  вроде проще так t = (plane.dist - Vector3.Dot(pos,plane.normal)) / Vector3.Dot(plane.normal,dir);
		t = plane.normal*(plane.normal*plane.dist - ray.pos) / (plane.normal*ray.dir);
		return t > 0;
	}

	template<class T, int Size>
	T SegmentSegmentDistance(TSegment<T, Size> s0, TSegment<T, Size> s1)
	{
		//http://www.softsurfer.com/Archive/algorithm_0106/algorithm_0106.htm#dist3D_Segment_to_Segment()
		const T SMALL_NUM = (T)0.000001;
		TVec<T, Size>   u = s0.p1 - s0.p0;
		TVec<T, Size>   v = s1.p1 - s1.p0;
		TVec<T, Size>   w = s0.p0 - s1.p0;
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
		else if (tN > tD)
		{      // tc > 1 => the t=1 edge is visible
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
		sc = (std::abs(sN) < SMALL_NUM ? (T) 0.0 : sN / sD);
		tc = (std::abs(tN) < SMALL_NUM ? (T) 0.0 : tN / tD);

		// get the difference of the two closest points
		TVec<T, Size>   dP = w + (u*sc) - (v*tc);  // = S1(sc) - S2(tc)

		return dP.Length();
	}

	template<class T, int Size>
	T SegmentRayDistance(TSegment<T, Size> s0, TRay<T, Size> s1, T& s0_t, T& s1_t)
	{
		//http://www.softsurfer.com/Archive/algorithm_0106/algorithm_0106.htm#dist3D_Segment_to_Segment()
		const T SMALL_NUM = (T)0.000001;
		TVec<T, Size>   u = s0.p1 - s0.p0;
		TVec<T, Size>   v = s1.dir;
		TVec<T, Size>   w = s0.p0 - s1.pos;
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

		// finally do the division to get sc and tc
		sc = (std::abs(sN) < SMALL_NUM ? (T) 0.0 : sN / sD);
		tc = (std::abs(tN) < SMALL_NUM ? (T)0.0 : tN / tD);

		s0_t = sc;
		s1_t = tc;

		// get the difference of the two closest points
		TVec<T, Size>   dP = w + (u*sc) - (v*tc);  // = S1(sc) - S2(tc)

		return dP.Length();
	}


	template<class T, int Size>
	T SegmentLineDistance(TSegment<T, Size> s0, TLine<T, Size> s1, T& s0_t, T& s1_t)
	{
		//http://www.softsurfer.com/Archive/algorithm_0106/algorithm_0106.htm#dist3D_Segment_to_Segment()
		const T SMALL_NUM = (T)0.000001;
		TVec<T, Size>   u = s0.p1 - s0.p0;
		TVec<T, Size>   v = s1.dir;
		TVec<T, Size>   w = s0.p0 - s1.p0;
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
		// finally do the division to get sc and tc
		sc = (std::abs(sN) < SMALL_NUM ? (T)0.0 : sN / sD);
		tc = (std::abs(tN) < SMALL_NUM ? (T)0.0 : tN / tD);

		s0_t = sc;
		s1_t = tc;

		// get the difference of the two closest points
		TVec<T, Size>   dP = w + (u*sc) - (v*tc);  // = S1(sc) - S2(tc)

		return dP.Length();
	}

	template<class T, int Size>
	T RayRayDistance(TRay<T, Size> s0, TRay<T, Size> s1)
	{
		//http://www.softsurfer.com/Archive/algorithm_0106/algorithm_0106.htm#dist3D_Segment_to_Segment()
		const T SMALL_NUM = 0.000001;
		TVec<T, Size>   u = s0.dir;
		TVec<T, Size>   v = s1.dir;
		TVec<T, Size>   w = s0.pos - s1.pos;
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

		// finally do the division to get sc and tc
		sc = (std::abs(sN) < SMALL_NUM ? 0.0 : sN / sD);
		tc = (std::abs(tN) < SMALL_NUM ? 0.0 : tN / tD);

		// get the difference of the two closest points
		TVec<T, Size>   dP = w + (u*sc) - (v*tc);  // = S1(sc) - S2(tc)

		return dP.Length();
	}



	struct TTri
	{
		int rib[3];
		bool inv_dir[3];
		TTri(){}
		TTri(int r0, int r1, int r2, bool i0, bool i1, bool i2)
		{
			rib[0] = r0;
			rib[1] = r1;
			rib[2] = r2;
			inv_dir[0] = i0;
			inv_dir[1] = i1;
			inv_dir[2] = i2;
		}
	};

	template<class T, int Size>
	void Tesselate(std::vector<TVec<T, Size> >& vertices, std::vector<TVec2ui>& ribs, std::vector<TTri>& triangles)
	{
		int ribs_high = ribs.size() - 1;
		ribs.resize(ribs.size() * 2);
		for (int i = ribs_high; i >= 0; i--)
		{
			TVec2ui rib = ribs[i];
			TVec<T, Size> middle = (vertices[rib[0]] + vertices[rib[1]])*0.5;
			vertices.push_back(middle);
			ribs[i * 2 + 0] = TVec2ui(rib[0], vertices.size() - 1);
			ribs[i * 2 + 1] = TVec2ui(vertices.size() - 1, rib[1]);
		}

		int triangles_high = triangles.size() - 1;
		triangles.resize(triangles.size() * 4);
		for (int i = triangles_high; i >= 0; i--)
		{
			TTri tri = triangles[i];

			//ребра треугольника посередине
			ribs.push_back(TVec2ui(ribs[tri.rib[0] * 2 + 0][1], ribs[tri.rib[1] * 2 + 0][1]));
			ribs.push_back(TVec2ui(ribs[tri.rib[1] * 2 + 0][1], ribs[tri.rib[2] * 2 + 0][1]));
			ribs.push_back(TVec2ui(ribs[tri.rib[2] * 2 + 0][1], ribs[tri.rib[0] * 2 + 0][1]));

			triangles[i * 4 + 0] = TTri(
				tri.rib[0] * 2 + tri.inv_dir[0],
				ribs.size() - 1 - 0,
				tri.rib[2] * 2 + !tri.inv_dir[2],
				tri.inv_dir[0], 1, tri.inv_dir[2]);
			triangles[i * 4 + 1] = TTri(
				tri.rib[0] * 2 + !tri.inv_dir[0],
				tri.rib[1] * 2 + tri.inv_dir[1],
				ribs.size() - 1 - 2,
				tri.inv_dir[0], tri.inv_dir[1], 1);
			triangles[i * 4 + 2] = TTri(
				tri.rib[1] * 2 + !tri.inv_dir[1],
				tri.rib[2] * 2 + tri.inv_dir[2],
				ribs.size() - 1 - 1,
				tri.inv_dir[1], tri.inv_dir[2], 1);
			triangles[i * 4 + 3] = TTri(
				ribs.size() - 1 - 2,
				ribs.size() - 1 - 1,
				ribs.size() - 1 - 0,
				0, 0, 0);
		}
	}

	template<class T>
	TVec<T, 2> LineLineIntersectionPoint(TSegment<T, 2> s0, TSegment<T, 2> s1)
	{
		float A1 = s0.p1[1] - s0.p0[1];
		float B1 = s0.p0[0] - s0.p1[0];
		float C1 = A1*s0.p0[0] + B1* s0.p0[1];

		// Get A,B,C of second line - points : ps2 to pe2
		float A2 = s1.p1[1] - s1.p0[1];
		float B2 = s1.p0[0] - s1.p1[0];
		float C2 = A2*s1.p0[0] + B2* s1.p0[1];

		// Get delta and check if the lines are parallel
		float delta = A1*B2 - A2*B1;
		//if (delta == 0)
		//	throw new System.Exception("Lines are parallel");

		// now return the Vector2 intersection point
		return TVec<T, 2>(
			(B2*C1 - B1*C2) / delta,
			(A1*C2 - A2*C1) / delta
			);
	}

	template <class T>
	bool SegmentSegmentCollide(TSegment<T, 2> s0, TSegment<T, 2> s1)
	{
		TVec<T, 2> r0 = (s0.p1 - s0.p0).Cross();
		TVec<T, 2> r1 = (s1.p1 - s1.p0).Cross();
		return (r0*s1.p0 > 0 != r0*s1.p1) && (r1*s0.p0 > 0 != r1*s0.p1);
	}

	//http://stackoverflow.com/questions/2049582/how-to-determine-a-point-in-a-triangle

	inline float sign(TVec2 p1, TVec2 p2, TVec2 p3)
	{
		return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1]);
	}

	inline bool PointInTriangle(TVec2 pt, TVec2 v1, TVec2 v2, TVec2 v3)
	{
		bool b1, b2, b3;

		b1 = sign(pt, v1, v2) < 0.0f;
		b2 = sign(pt, v2, v3) < 0.0f;
		b3 = sign(pt, v3, v1) < 0.0f;

		return ((b1 == b2) && (b2 == b3));
	}

	namespace MathUtils
	{

		template <class T>
		T Area(TVec<T, 2> a, TVec<T, 2> b, TVec<T, 2> c)
		{
			return a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]);
		}
		template <class T>
		T VectorAngle(TVec<T, 2> p1, TVec<T, 2> p2)
		{
			T theta1 = atan2(p1[1], p1[0]);
			T theta2 = atan2(p2[1], p2[0]);
			T dtheta = theta2 - theta1;
			while (dtheta > M_PI)
				dtheta -= (2 * M_PI);
			while (dtheta < -M_PI)
				dtheta += (2 * M_PI);

			return (dtheta);
		}
	}
}