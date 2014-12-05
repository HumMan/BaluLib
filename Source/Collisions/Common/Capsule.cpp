#include "../../BVolumes/Capsule.h"

template<class T,int Size>
bool TCapsule<T,Size>::Contain(const TVec<T,Size>& point) const
{
	T t;
	TVec<T,Size> nearest_point;
	return DistanceBetweenPointLine<T, Size>(point, segment.p0, segment.p1, t, nearest_point)<radius;
}

template<class T,int Size>
bool TCapsule<T,Size>::Contain(const TVec<T,Size>& point,T& distance, TVec<T,Size>& nearest_point, TVec<T,Size>& normal) const
{
	T t;
	distance = DistanceBetweenPointLine<T, Size>(point, segment.p0, segment.p1, t, nearest_point);
	bool result=distance<radius;
	normal=(point-nearest_point)*(1/distance);
	return result;
}


template<class T, int Size>
bool TCapsule<T, Size>::CollideWith(const TBVolume<T, Size>& v)const							{ return v.CollideWith(*this); }
template<class T, int Size>
bool TCapsule<T, Size>::CollideWith(const TFrustum<T, Size>& frustum)const						{ return frustum.Overlaps(*this); }
template<class T, int Size>
bool TCapsule<T, Size>::CollideWith(const TFrustum<T, Size>& frustum, bool& fully_in_frustum)const{ return frustum.Overlaps(*this, fully_in_frustum); }
template<class T, int Size>
bool TCapsule<T, Size>::CollideWith(const TAABB<T, Size>& v)const								{ return Collide<T, Size>(*this, v); }
template<class T, int Size>
bool TCapsule<T, Size>::CollideWith(const TOBB<T, Size>& v)const								{ return Collide<T, Size>(*this, v); }
template<class T, int Size>
bool TCapsule<T, Size>::CollideWith(const TCapsule<T, Size>& v) const							{ return Collide<T, Size>(*this, v); }
template<class T, int Size>
bool TCapsule<T, Size>::CollideWith(const TSphere<T, Size>& v) const							{ return Collide<T, Size>(*this, v); }

template<class T,int Size>
bool TCapsule<T,Size>::CollideWith(const TRay<T,Size> &ray) const
{
	T ray_t, capsule_t;
	return SegmentRayDistance(segment, ray, capsule_t, ray_t)<radius;
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
	T ray_t, capsule_t;
	T dist = SegmentRayDistance(segment, ray, capsule_t, ray_t);
	//TVec<T, Size> ray_pos = ray.pos + ray.dir*ray_t;
	//TVec<T, Size> seg_pos = segment.p0 + (segment.p1 - segment.p0).GetNormalized()*capsule_t;
	//normal0 = (ray_pos - seg_pos).GetNormalized();
	return dist<radius;
}


template<class T>
void DrawTrianglesSpecialized(const TCapsule<T, 2>& capsule, std::vector<TVec<T, 2> >& vertices, std::vector<unsigned int>& indices)
{
}

template<class T>
void DrawTrianglesSpecialized(const TCapsule<T, 3>& capsule, std::vector<TVec<T, 3> >& vertices, std::vector<unsigned int>& indices)
{
	std::vector<TVec2ui> ribs;
	std::vector<TTri> triangles;

	int vertices_first = vertices.size();
	vertices.resize(vertices.size() + 5);
	vertices[vertices_first + 0] = TVec<T, 3>(0, 0, capsule.radius);		//0
	vertices[vertices_first + 1] = TVec<T, 3>(-capsule.radius, 0, 0);		//1
	vertices[vertices_first + 2] = TVec<T, 3>(0, 0, -capsule.radius);		//2
	vertices[vertices_first + 3] = TVec<T, 3>(capsule.radius, 0, 0);		//3
	vertices[vertices_first + 4] = TVec<T, 3>(0, capsule.radius, 0);		//4
	//
	ribs.resize(8);
	ribs[0] = TVec2ui(0, 4);		//0
	ribs[1] = TVec2ui(1, 4);		//1
	ribs[2] = TVec2ui(2, 4);		//2
	ribs[3] = TVec2ui(3, 4);		//3
	ribs[4] = TVec2ui(0, 1);		//4
	ribs[5] = TVec2ui(1, 2);		//5
	ribs[6] = TVec2ui(2, 3);		//6
	ribs[7] = TVec2ui(3, 0);		//7
	for (int i = 0; i <= 7; i++)ribs[i] += TVec2ui(vertices_first);
	//
	triangles.resize(4);
	triangles[0] = TTri(0, 1, 4, 0, 1, 1);
	triangles[1] = TTri(7, 3, 0, 1, 0, 1);
	triangles[2] = TTri(2, 5, 1, 1, 1, 0);
	triangles[3] = TTri(6, 2, 3, 1, 0, 1);

	for (int i = 0; i<4; i++)
		Tesselate(vertices, ribs, triangles);

	TMatrix<T, 3> orient = capsule.GetOrientation();

	for (int i = vertices_first; i<vertices.size(); i++)
		vertices[i] = capsule.segment.p0 + orient*vertices[i];

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
void TCapsule<T, Size>::DrawTriangles(std::vector<TVec<T, Size> >& vertices, std::vector<unsigned int>& indices)const
{
	DrawTrianglesSpecialized(*this, vertices, indices);
}

template<class T>
void DrawLinesSpecialized(const TCapsule<T, 2>& capsule, std::vector<TVec<T, 3> >& vertices)
{
	//const int step=20;
	//const int v_count=int(360/step)+1;
	//for(int i=0;i<v_count;i++)
	//{
	//	vertices.Push(TVec<T,Size>(float(radius*sin(i*step*M_PI/180)),float(radius*cos(i*step*M_PI/180)))+pos);
	//	vertices.Push(TVec<T,Size>(float(radius*sin((i+1)*step*M_PI/180)),float(radius*cos((i+1)*step*M_PI/180)))+pos);
	//}
}

template<class T>
void DrawLinesSpecialized(const TCapsule<T, 3>& capsule, std::vector<TVec<T, 3> >& vertices)
{
	int v_count = 20;
	T step = M_PI / v_count;
	T alpha0, alpha1;
	int vertices_last_high = vertices.size();
	for (int i = 0; i<v_count; i++)
	{
		alpha0 = i*step + (T)(M_PI*0.5);
		alpha1 = alpha0 + step;
		vertices.push_back(TVec<T, 3>(capsule.radius*cos(alpha0), capsule.radius*sin(alpha0), 0));
		vertices.push_back(TVec<T, 3>(capsule.radius*cos(alpha1), capsule.radius*sin(alpha1), 0));
	}
	for (int i = 0; i<v_count; i++)
	{
		alpha0 = i*step + (T)(M_PI*0.5);
		alpha1 = alpha0 + step;
		vertices.push_back(TVec<T, 3>(capsule.radius*cos(alpha0), 0, capsule.radius*sin(alpha0)));
		vertices.push_back(TVec<T, 3>(capsule.radius*cos(alpha1), 0, capsule.radius*sin(alpha1)));
	}

	int vertices_size = vertices.size();
	T size = capsule.segment.p0.Distance(capsule.segment.p1);
	for (int i = vertices_last_high; i < vertices_size; i++)
	{
		TVec<T, 3> temp(vertices[i]);
		temp[0] = size - temp[0];
		vertices.push_back(temp);
	}

	v_count = v_count * 2;
	step = 2 * (T)M_PI / v_count;
	for (int i = 0; i<v_count; i++)
	{
		alpha0 = i*step;
		alpha1 = alpha0 + step;
		vertices.push_back(TVec<T, 3>(0, capsule.radius*cos(alpha0), capsule.radius*sin(alpha0)));
		vertices.push_back(TVec<T, 3>(0, capsule.radius*cos(alpha1), capsule.radius*sin(alpha1)));
		vertices.push_back(TVec<T, 3>(size, capsule.radius*cos(alpha0), capsule.radius*sin(alpha0)));
		vertices.push_back(TVec<T, 3>(size, capsule.radius*cos(alpha1), capsule.radius*sin(alpha1)));
	}

	v_count = 4;
	step = 2 * (T)M_PI / v_count;

	for (int i = 0; i<v_count; i++)
	{
		alpha0 = i*step;
		alpha1 = alpha0 + step;
		vertices.push_back(TVec<T, 3>(0, capsule.radius*cos(alpha1), capsule.radius*sin(alpha1)));
		vertices.push_back(TVec<T, 3>(size, capsule.radius*cos(alpha1), capsule.radius*sin(alpha1)));
	}

	TMatrix<T, 3> orient = capsule.GetOrientation();

	for (int i = vertices_last_high; i<vertices.size(); i++)
		vertices[i] = capsule.segment.p0 + orient*vertices[i];
}

template<class T, int Size>
void TCapsule<T, Size>::DrawLines(std::vector<TVec<T, Size> >& vertices)const
{
	DrawLinesSpecialized(*this, vertices);
}


template class TCapsule<float, 2>;
template class TCapsule<float, 3>;
template class TCapsule<double, 2>;
template class TCapsule<double, 3>;