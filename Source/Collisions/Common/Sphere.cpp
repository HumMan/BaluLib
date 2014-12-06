#include "../../BVolumes/Sphere.h"

#include "../SphereAndSphere.h"
#include "../OBBAndSphere.h"
#include "../CapsuleAndSphere.h"
#include "../AABBAndSphere.h"

#include "../../BVolumes/Frustum.h"

template<class T, int Size>
bool TSphere<T, Size>::PointCollide(const TVec<T, Size>& point) const
{
	return point.SqrDistance(pos)<radius*radius;
}

template<class T, int Size>
bool TSphere<T, Size>::PointCollide(const TVec<T, Size>& point, TPointCollisionInfo<T, Size>& collision) const
{
	TVec<T, Size> temp = point - pos;
	collision.normal = temp.GetNormalized();
	collision.nearest_point = collision.normal*radius;
	return temp.SqrLength()<radius*radius;
}

template<class T, int Size>
bool TSphere<T, Size>::RayCollide(const TRay<T, Size> &ray) const
{

	TVec<T, Size> diff = ray.pos - pos;
	T a0 = diff*diff - sqr(radius);
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

template<class T, int Size>
bool TSphere<T, Size>::RayCollide(const TRay<T, Size> &ray, TRayCollisionInfo<T, Size>& collision) const
{
	T a, b, c;
	TVec<T, Size> t, d;
	d = ray.pos - pos;
	a = (ray.dir*ray.dir);
	b = (ray.dir*d)*2.0f;
	c = (d*d - radius*radius);
	T discr = b*b - 4 * a*c;
	if (discr<0)return false;
	discr = sqrt(discr);
	T t0 = (b + discr) / (-2 * a);
	T t1 = (b - discr) / (-2 * a);
	assert(t1 >= t0);
	if (t0<0 && t1<0)return false;
	return true;
}

//template<class T, int Size>
//bool TSphere<T, Size>::CollideWith(const TRay<T, Size> &ray, T& t, TVec<T, Size>& normal) const
//{
//	T t0, t1;
//	if (CollideWith(ray, t0, t1))
//	{
//		T t = t0>0 ? t0 : t1;
//		normal = (ray.pos + ray.dir*t - pos).GetNormalized();
//		return true;
//	}
//	else return false;
//}

template<class T, int Size>
bool TSphere<T, Size>::PlaneCollide(const TPlane<T, Size> &plane) const
{
	return false;//TODO
}
template<class T, int Size>
bool TSphere<T, Size>::PlaneCollide(const TPlane<T, Size> &plane, TPlaneCollisionInfo<T, Size>& collision) const
{
	return false;//TODO
}
template<class T, int Size>
bool TSphere<T, Size>::SegmentCollide(const TSegment<T, Size> &segment) const
{
	return false;//TODO
}
template<class T, int Size>
bool TSphere<T, Size>::SegmentCollide(const TSegment<T, Size> &segment, TRayCollisionInfo<T, Size>& collision) const
{
	return false;//TODO
}
template<class T, int Size>
bool TSphere<T, Size>::LineCollide(const TLine<T, Size> &line) const
{
	return false;//TODO
}
template<class T, int Size>
bool TSphere<T, Size>::LineCollide(const TLine<T, Size> &line, TRayCollisionInfo<T, Size>& collision) const
{
	return false;//TODO
}


template<class T>
void DrawTrianglesSpecialized(const TSphere<T, 2>& sphere, std::vector<TVec<T, 2> >& vertices, std::vector<unsigned int>& indices)
{
}

template<class T>
void DrawTrianglesSpecialized(const TSphere<T, 3>& sphere, std::vector<TVec<T, 3> >& vertices, std::vector<unsigned int>& indices)
{
	std::vector<TVec2ui> ribs;
	std::vector<TTri> triangles;

	int vertices_first = vertices.size();
	vertices.resize(vertices.size() + 6);
	vertices[vertices_first + 0] = TVec<T, 3>(0, 0, sphere.radius);		//0
	vertices[vertices_first + 1] = TVec<T, 3>(-sphere.radius, 0, 0);		//1
	vertices[vertices_first + 2] = TVec<T, 3>(0, 0, -sphere.radius);		//2
	vertices[vertices_first + 3] = TVec<T, 3>(sphere.radius, 0, 0);		//3
	vertices[vertices_first + 4] = TVec<T, 3>(0, sphere.radius, 0);		//4
	vertices[vertices_first + 5] = TVec<T, 3>(0, -sphere.radius, 0);		//5
	//
	ribs.resize(12);
	ribs[0] = TVec2ui(0, 4);		//0
	ribs[1] = TVec2ui(1, 4);		//1
	ribs[2] = TVec2ui(2, 4);		//2
	ribs[3] = TVec2ui(3, 4);		//3
	ribs[4] = TVec2ui(0, 1);		//4
	ribs[5] = TVec2ui(1, 2);		//5
	ribs[6] = TVec2ui(2, 3);		//6
	ribs[7] = TVec2ui(3, 0);		//7
	ribs[8] = TVec2ui(0, 5);		//8
	ribs[9] = TVec2ui(1, 5);		//9
	ribs[10] = TVec2ui(2, 5);		//10
	ribs[11] = TVec2ui(3, 5);		//11
	for (int i = 0; i<12; i++)ribs[i] += TVec2ui(vertices_first);
	//
	triangles.resize(8);
	triangles[0] = TTri(0, 1, 4, 0, 1, 1);
	triangles[1] = TTri(7, 3, 0, 1, 0, 1);
	triangles[2] = TTri(8, 11, 7, 0, 1, 0);
	triangles[3] = TTri(4, 9, 8, 0, 0, 1);
	triangles[4] = TTri(5, 10, 9, 0, 0, 1);
	triangles[5] = TTri(2, 5, 1, 1, 1, 0);
	triangles[6] = TTri(6, 2, 3, 1, 0, 1);
	triangles[7] = TTri(10, 6, 11, 1, 0, 0);

	for (int i = 0; i<4; i++)
		Tesselate(vertices, ribs, triangles);

	for (int i = vertices_first; i<vertices.size(); i++)
		vertices[i] = sphere.pos + (vertices[i]).GetNormalized()*sphere.radius;

	int indices_first = indices.size();
	indices.resize(indices.size() + triangles.size() * 3);
	for (int i = 0; i<triangles.size(); i++)
	{
		indices[indices_first + i * 3 + 0] = ribs[triangles[i].rib[0]][triangles[i].inv_dir[0]];
		indices[indices_first + i * 3 + 1] = ribs[triangles[i].rib[1]][triangles[i].inv_dir[1]];
		indices[indices_first + i * 3 + 2] = ribs[triangles[i].rib[2]][triangles[i].inv_dir[2]];
	}
}

template<class T, int Size>
void TSphere<T, Size>::DrawTriangles(std::vector<TVec<T, Size> >& vertices, std::vector<unsigned int>& indices)const
{
	DrawTrianglesSpecialized(*this, vertices, indices);
}

template<class T>
void DrawLinesSpecialized(const TSphere<T, 2>& sphere, std::vector<TVec<T, 2> >& vertices)
{
	const int step = 20;
	const int v_count = int(360 / step) + 1;
	for (int i = 0; i<v_count; i++)
	{
		vertices.push_back(TVec<T, 2>(float(sphere.radius*sin(i*step*M_PI / 180)), float(sphere.radius*cos(i*step*M_PI / 180))) + sphere.pos);
		vertices.push_back(TVec<T, 2>(float(sphere.radius*sin((i + 1)*step*M_PI / 180)), float(sphere.radius*cos((i + 1)*step*M_PI / 180))) + sphere.pos);
	}
}

template<class T>
void DrawLinesSpecialized(const TSphere<T, 3>& sphere, std::vector<TVec<T, 3> >& vertices)
{
	//const int step=20;
	//const int v_count=int(360/step)+1;
	//for(int i=0;i<v_count;i++)
	//{
	//	vertices.Push(TVec<T,Size>(float(radius*sin(i*step*M_PI/180)),float(radius*cos(i*step*M_PI/180)),0.0f)+pos);
	//	vertices.Push(TVec<T,Size>(float(radius*sin((i+1)*step*M_PI/180)),float(radius*cos((i+1)*step*M_PI/180)),0.0f)+pos);
	//}
	//for(int i=0;i<v_count;i++)
	//{
	//	vertices.Push(TVec<T,Size>(0.0f,float(radius*sin(i*step*M_PI/180)),float(radius*cos(i*step*M_PI/180)))+pos);
	//	vertices.Push(TVec<T,Size>(0.0f,float(radius*sin((i+1)*step*M_PI/180)),float(radius*cos((i+1)*step*M_PI/180)))+pos);
	//}
	//for(int i=0;i<v_count;i++)
	//{
	//	vertices.Push(TVec<T,Size>(float(radius*sin(i*step*M_PI/180)),0.0f,float(radius*cos(i*step*M_PI/180)))+pos);
	//	vertices.Push(TVec<T,Size>(float(radius*sin((i+1)*step*M_PI/180)),0.0f,float(radius*cos((i+1)*step*M_PI/180)))+pos);
	//}

	int v_count = 20;
	T step = 2 * M_PI / v_count;
	T alpha0, alpha1;
	int vertices_last_size = vertices.size();
	for (int i = 0; i<v_count; i++)
	{
		alpha0 = i*step;
		alpha1 = alpha0 + step;
		vertices.push_back(TVec<T, 3>(sphere.radius*cos(alpha0), sphere.radius*sin(alpha0), 0));
		vertices.push_back(TVec<T, 3>(sphere.radius*cos(alpha1), sphere.radius*sin(alpha1), 0));
	}
	for (int i = 0; i<v_count; i++)
	{
		alpha0 = i*step;
		alpha1 = alpha0 + step;
		vertices.push_back(TVec<T, 3>(sphere.radius*cos(alpha0), 0, sphere.radius*sin(alpha0)));
		vertices.push_back(TVec<T, 3>(sphere.radius*cos(alpha1), 0, sphere.radius*sin(alpha1)));
	}
	for (int i = 0; i<v_count; i++)
	{
		alpha0 = i*step;
		alpha1 = alpha0 + step;
		vertices.push_back(TVec<T, 3>(0, sphere.radius*cos(alpha0), sphere.radius*sin(alpha0)));
		vertices.push_back(TVec<T, 3>(0, sphere.radius*cos(alpha1), sphere.radius*sin(alpha1)));
	}
	for (int i = vertices_last_size; i<vertices.size(); i++)
		vertices[i] = sphere.pos + vertices[i];
}

template<class T, int Size>
void TSphere<T, Size>::DrawLines(std::vector<TVec<T, Size> >& vertices)const
{
	DrawLinesSpecialized(*this, vertices);
}



template<class T, int Size>
bool TSphere<T, Size>::CollideWith(const TBVolume<T, Size>& v)const
{
	return v.CollideWith(*this);
}
template<class T, int Size>
bool TSphere<T, Size>::CollideWith(const TBVolume<T, Size>& v, bool& fully_in_volume)const
{
	return v.CollideWith(*this, fully_in_volume);
}

template<class T>
bool CollideWithSpecialized(const TSphere<T, 2>& aabb, const TFrustum<T, 2>& frustum)
{
	//static_assert(false, "supports only 3d");
	return false;
}
template<class T>
bool CollideWithSpecialized(const TSphere<T, 3>& aabb, const TFrustum<T, 3>& frustum)
{
	return frustum.Overlaps(aabb);
}
template<class T, int Size>
bool TSphere<T, Size>::CollideWith(const TFrustum<T, Size>& frustum)const
{
	return CollideWithSpecialized(*this, frustum);
}

template<class T>
bool CollideWithSpecialized(const TSphere<T, 2>& aabb, const TFrustum<T, 2>& frustum, bool& fully_in_frustum)
{
	//static_assert(false, "supports only 3d");
	return false;
}
template<class T>
bool CollideWithSpecialized(const TSphere<T, 3>& aabb, const TFrustum<T, 3>& frustum, bool& fully_in_frustum)
{
	return frustum.Overlaps(aabb, fully_in_frustum);
}
template<class T, int Size>
bool TSphere<T, Size>::CollideWith(const TFrustum<T, Size>& frustum, bool& fully_in_frustum)const
{
	return CollideWithSpecialized(*this, frustum, fully_in_frustum);
}

template<class T, int Size>
bool TSphere<T, Size>::CollideWith(const TAABB<T, Size>& v)const
{
	return Collide<T, Size>(v, *this);
}
template<class T, int Size>
bool TSphere<T, Size>::CollideWith(const TAABB<T, Size>& v, bool& fully_in_aabb)const
{
	return Collide<T, Size>(*this, v, fully_in_aabb);
}
template<class T, int Size>
bool TSphere<T, Size>::CollideWith(const TOBB<T, Size>& v)const
{
	return Collide<T, Size>(*this, v);
}
template<class T, int Size>
bool TSphere<T, Size>::CollideWith(const TOBB<T, Size>& v, bool& fully_in_obb)const
{
	return Collide<T, Size>(v, *this, fully_in_obb);
}
template<class T, int Size>
bool TSphere<T, Size>::CollideWith(const TCapsule<T, Size>& v)const
{
	return Collide(v, *this);
}
template<class T, int Size>
bool TSphere<T, Size>::CollideWith(const TCapsule<T, Size>& v, bool& fully_in_capsule)const
{
	return Collide(v, *this, fully_in_capsule);
}
template<class T, int Size>
bool TSphere<T, Size>::CollideWith(const TSphere<T, Size>& v)const
{
	return Collide<T, Size>(v, *this);
}
template<class T, int Size>
bool TSphere<T, Size>::CollideWith(const TSphere<T, Size>& v, bool& fully_in_sphere)const
{
	return Collide<T, Size>(*this, v, fully_in_sphere);
}



template class TSphere<float, 2>;
template class TSphere<float, 3>;
template class TSphere<double, 2>;
template class TSphere<double, 3>;