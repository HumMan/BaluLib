#include "../../BVolumes/AABB.h"

#include "../AABBAndAABB.h"
#include "../AABBAndSphere.h"
#include "../CapsuleAndAABB.h"
#include "../OBBAndAABB.h"

#include "../../BVolumes/Frustum.h"

template<class T,int Size>
bool TAABB<T,Size>::PointCollide(const TVec<T,Size>& point) const
{
	for(int i=0;i<Size;i++)
	{
		if(!IsInMinMax(point[i],border[0][i],border[1][i]))
			return false;
	}
	return true;
}

template<class T,int Size>
bool TAABB<T, Size>::PointCollide(const TVec<T, Size>& point, TPointCollisionInfo<T, Size>& collision) const
{
	bool is_in = true;
	TVec<T, Size> nearest_point;
	for(int i=0;i<Size;i++)
	{
		if(point[i]>border[1][i])
		{
			nearest_point[i]=border[1][i];
			is_in = false;
		}else if(point[i]<border[0][i])
		{
			nearest_point[i]=border[0][i];
			is_in = false;
		}
		else if (abs(point[i] - border[0][i])>abs(point[i] - border[1][i]))
			nearest_point[i] = border[1][i];
		else
			nearest_point[i] = border[0][i];
	}
	collision.nearest_point = nearest_point;
	collision.normal = nearest_point - point;
	collision.distance = collision.normal.Length();
	collision.normal *= (T)1/collision.distance;
	collision.is_in = is_in;
	return is_in;
}


template<class T, int Size>
bool TAABB<T, Size>::RayCollide(const TRay<T, Size> &ray) const
{
	static_assert(Size >= 2 && Size <= 3, "only 2d 3d support");
	if (Size == 2)
	{
		if (ray.dir[0]>0){
			if (ray.pos[0]>border[1][0])return false;
		}
		else if (ray.pos[0]<border[0][0])return false;
		if (ray.dir[1]>0){
			if (ray.pos[1]>border[1][1])return false;
		}
		else if (ray.pos[1]<border[0][1])return false;
		TVec<T, Size>
			size = GetSize(),
			v = GetCenter() - ray.pos;
		if (abs(v[1] * ray.dir[0] - v[0] * ray.dir[1])>size[0] * abs(ray.dir[1]) + size[1] * abs(ray.dir[0]))
			return false;
		return true;
	}
	if (Size == 3)
	{
		if (ray.dir[0]>0){
			if (ray.pos[0]>border[1][0])return false;
		}
		else if (ray.pos[0]<border[0][0])return false;
		if (ray.dir[1]>0){
			if (ray.pos[1]>border[1][1])return false;
		}
		else if (ray.pos[1]<border[0][1])return false;
		if (ray.dir[2]>0){
			if (ray.pos[2]>border[1][2])return false;
		}
		else if (ray.pos[2]<border[0][2])return false;
		TVec<T, Size>
			size = GetSize(),
			v = GetCenter() - ray.pos;
		if (abs(v[2] * ray.dir[1] - v[1] * ray.dir[2])>size[1] * abs(ray.dir[2]) + size[2] * abs(ray.dir[1]))
			return false;
		if (abs(v[0] * ray.dir[2] - v[2] * ray.dir[0])>size[0] * abs(ray.dir[2]) + size[2] * abs(ray.dir[0]))
			return false;
		if (abs(v[0] * ray.dir[1] - v[1] * ray.dir[0])>size[0] * abs(ray.dir[1]) + size[1] * abs(ray.dir[0]))
			return false;
		return true;
	}
	return true;
}

template<class T>
bool CollideAxis(T start, T first, T last, T dir, T& cf, T& cl, bool& f_curr_cut, bool& f_cut_min, bool& l_curr_cut, bool& l_cut_min)
{
	f_curr_cut = false;
	l_curr_cut = false;
	T f = (first - start) / dir;
	T l = (last - start) / dir;
	if (f<0 && l<0)return false;
	if (dir >= 0)
	{
		if (f>cf)
		{
			f_curr_cut = true;
			f_cut_min = true;
			cf = f;
		}
		if (l<cl)
		{
			l_curr_cut = true;
			l_cut_min = false;
			cl = l;
		}
	}
	else
	{
		if (l>cf)
		{
			f_curr_cut = true;
			f_cut_min = false;
			cf = l;
		}
		if (f<cl)
		{
			l_curr_cut = true;
			l_cut_min = true;
			cl = f;
		}
	}
	if (cf>cl)return false;
	return true;
}

template<class T, int Size>
bool TAABB<T, Size>::RayCollide(const TRay<T, Size> &ray, TRayCollisionInfo<T, Size>& collision) const
{
	bool result = true;
	T cf = 0, cl = 10e30;
	int fcut = -1, lcut = -1;
	bool fmin = false, lmin = false;
	for (int i = 0; i<Size&&result; i++)
	{
		bool f_curr_cut, f_cut_min;
		bool l_curr_cut, l_cut_min;
		result = result&&CollideAxis<T>(ray.pos[i], border[0][i], border[1][i], ray.dir[i], cf, cl, f_curr_cut, f_cut_min, l_curr_cut, l_cut_min);
		if (f_curr_cut)
		{
			fcut = i;
			fmin = f_cut_min;
		}
		if (l_curr_cut)
		{
			lcut = i;
			lmin = l_cut_min;
		}
	}
	if (fcut != -1)
	{
		collision.have_in = true;
		collision.in_param = cf;
		TVec<T, Size> normal(0);
		normal[fcut] = fmin ? -1 : 1;
		collision.in_normal = normal;
	}else
		collision.have_in = false;

	if (lcut != -1)
	{
		collision.have_out = true;
		collision.out_param = cl;
		TVec<T, Size> normal(0);
		normal[lcut] = lmin ? -1 : 1;
		collision.in_normal = normal;
	}
	else
		collision.have_out = false;
	collision.param_direction = ray.dir;
	return result;
}

template<class T, int Size>
bool TAABB<T, Size>::PlaneCollide(const TPlane<T, Size> &plane) const 
{
	TVec<T, Size> plane_pos = plane.normal*(-plane.dist);
	TVec<T, Size> aabb_pos = this->GetCenter();
	TVec<T, Size> aabb_size = this->GetSize();

	return (plane_pos - aabb_pos)*plane.normal < aabb_size.AbsScalarMul(plane.normal);
}

template<class T, int Size>
bool TAABB<T, Size>::PlaneCollide(const TPlane<T, Size> &plane, TPlaneCollisionInfo<T, Size>& collision) const 
{
	TVec<T, Size> plane_pos = plane.normal*(-plane.dist);
	//TVec<T, Size> proj_dir = plane.dist >= 0 ? plane.normal : -plane.normal;
	TVec<T, Size> aabb_pos = this->GetCenter();
	TVec<T, Size> aabb_size = this->GetSize();
	TVec<T, Size> tested_dir = plane_pos - aabb_pos;
	//find nearest point on box
	TVec<T, Size> nearest_point;
	for (int i = 0; i < Size; i++)
	{
		if (tested_dir[i]>0)
			nearest_point[i] = border[1][i];
		else
			nearest_point[i] = border[0][i];
	}

	collision.nearest_point = nearest_point;
	collision.distance = (plane_pos - nearest_point)*plane.normal;
	//TODO
	collision.normal = plane.normal;
	return collision.distance < 0;
}

template<class T, int Size>
bool TAABB<T, Size>::SegmentCollide(const TSegment<T, Size> &segment) const 
{
}
template<class T, int Size>
bool TAABB<T, Size>::SegmentCollide(const TSegment<T, Size> &segment, TRayCollisionInfo<T, Size>& collision) const 
{
}
template<class T, int Size>
bool TAABB<T, Size>::LineCollide(const TLine<T, Size> &line) const  
{
}
template<class T, int Size>
bool TAABB<T, Size>::LineCollide(const TLine<T, Size> &line, TRayCollisionInfo<T, Size>& collision) const  
{
}

const bool quads_of_box[6][4][3] =
{
	{ { 0, 0, 0 }, { 0, 0, 1 }, { 0, 1, 1 }, { 0, 1, 0 } },  //quad 1
	{ { 1, 0, 0 }, { 1, 1, 0 }, { 1, 1, 1 }, { 1, 0, 1 } },  //quad 2
	{ { 0, 0, 0 }, { 1, 0, 0 }, { 1, 0, 1 }, { 0, 0, 1 } },  //quad 3
	{ { 0, 1, 0 }, { 0, 1, 1 }, { 1, 1, 1 }, { 1, 1, 0 } },  //quad 4
	{ { 0, 0, 0 }, { 0, 1, 0 }, { 1, 1, 0 }, { 1, 0, 0 } },  //quad 5
	{ { 0, 0, 1 }, { 1, 0, 1 }, { 1, 1, 1 }, { 0, 1, 1 } }   //quad 6
};

const bool tri_ribs_of_box[24][3] =
{
	{ 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 1, 1, 0 }, { 0, 1, 1 }, { 1, 1, 1 }, { 0, 0, 1 }, { 1, 0, 1 },
	{ 0, 0, 0 }, { 0, 1, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 1, 0, 1 }, { 1, 1, 1 }, { 0, 0, 1 }, { 0, 1, 1 },
	{ 0, 0, 0 }, { 0, 0, 1 }, { 1, 0, 0 }, { 1, 0, 1 }, { 1, 1, 0 }, { 1, 1, 1 }, { 0, 1, 0 }, { 0, 1, 1 },
};


template<class T>
void DrawTrianglesSpecialized(TAABB<T, 3> aabb, std::vector<TVec<T, 3> >& vertices, std::vector<unsigned int>& indices)
{
	int vertices_first = vertices.size();

	for (int i = 7; i >= 0; i--)
		vertices.push_back(TVec<T, 3>(aabb.border[(i >> 2) & 1][0], aabb.border[(i >> 1) & 1][1], aabb.border[i & 1][2]));
	indices.resize(indices.size() + 6 * 3 * 2);
	for (int i = 0; i < 6; i++)
	{
		indices[indices.size() - 1 - (i * 3 * 2 + 0)] = vertices_first + (quads_of_box[i][0][2] << 2) + (quads_of_box[i][0][1] << 1) + quads_of_box[i][0][0];
		indices[indices.size() - 1 - (i * 3 * 2 + 1)] = vertices_first + (quads_of_box[i][1][2] << 2) + (quads_of_box[i][1][1] << 1) + quads_of_box[i][1][0];
		indices[indices.size() - 1 - (i * 3 * 2 + 2)] = vertices_first + (quads_of_box[i][2][2] << 2) + (quads_of_box[i][2][1] << 1) + quads_of_box[i][2][0];

		indices[indices.size() - 1 - (i * 3 * 2 + 3)] = vertices_first + (quads_of_box[i][0][2] << 2) + (quads_of_box[i][0][1] << 1) + quads_of_box[i][0][0];
		indices[indices.size() - 1 - (i * 3 * 2 + 4)] = vertices_first + (quads_of_box[i][3][2] << 2) + (quads_of_box[i][3][1] << 1) + quads_of_box[i][3][0];
		indices[indices.size() - 1 - (i * 3 * 2 + 5)] = vertices_first + (quads_of_box[i][2][2] << 2) + (quads_of_box[i][2][1] << 1) + quads_of_box[i][2][0];
	}
}

template<class T>
void DrawTrianglesSpecialized(TAABB<T, 2> aabb, std::vector<TVec<T, 2> >& vertices, std::vector<unsigned int>& indices)
{
	vertices.push_back(TVec<T, 2>(aabb.border[0][0], aabb.border[0][1]));
	vertices.push_back(TVec<T, 2>(aabb.border[0][0], aabb.border[1][1]));
	vertices.push_back(TVec<T, 2>(aabb.border[1][0], aabb.border[1][1]));
	vertices.push_back(TVec<T, 2>(aabb.border[1][0], aabb.border[0][1]));

	indices.push_back(vertices.size() - 4);
	indices.push_back(vertices.size() - 3);
	indices.push_back(vertices.size() - 2);

	indices.push_back(vertices.size() - 4);
	indices.push_back(vertices.size() - 1);
	indices.push_back(vertices.size() - 2);
}

template<class T, int Size>
void TAABB<T, Size>::DrawTriangles(std::vector<TVec<T, Size> >& vertices, std::vector<unsigned int>& indices)const
{
	static_assert(Size >= 2 && Size <= 3, "only 2d 3d support");
	DrawTrianglesSpecialized(*this, vertices, indices);
}


template<class T>
void DrawLinesSpecialized(TAABB<T, 2> aabb, std::vector<TVec<T, 2> >& vertices)
{
	vertices.push_back(TVec<T, 2>(aabb.border[0][0], aabb.border[0][1]));
	vertices.push_back(TVec<T, 2>(aabb.border[1][0], aabb.border[0][1]));

	vertices.push_back(TVec<T, 2>(aabb.border[0][0], aabb.border[0][1]));
	vertices.push_back(TVec<T, 2>(aabb.border[0][0], aabb.border[1][1]));

	vertices.push_back(TVec<T, 2>(aabb.border[0][0], aabb.border[1][1]));
	vertices.push_back(TVec<T, 2>(aabb.border[1][0], aabb.border[1][1]));

	vertices.push_back(TVec<T, 2>(aabb.border[1][0], aabb.border[0][1]));
	vertices.push_back(TVec<T, 2>(aabb.border[1][0], aabb.border[1][1]));
}

template<class T>
void DrawLinesSpecialized(TAABB<T, 3> aabb, std::vector<TVec<T, 3> >& vertices)
{
	for (int i = 0; i<24; i++)
		vertices.push_back(TVec<T, 3>
		(
		aabb.border[tri_ribs_of_box[i][0]][0],
		aabb.border[tri_ribs_of_box[i][1]][1],
		aabb.border[tri_ribs_of_box[i][2]][2]
		));
}

template<class T, int Size>
void TAABB<T, Size>::DrawLines(std::vector<TVec<T, Size> >& vertices)const
{
	static_assert(Size >= 2 && Size <= 3, "only 2d 3d support");
	DrawLinesSpecialized(*this, vertices);
}


template<class T, int Size>
bool TAABB<T, Size>::CollideWith(const TBVolume<T, Size>& v)const
{
	return v.CollideWith(*this);
}
template<class T, int Size>
bool TAABB<T, Size>::CollideWith(const TBVolume<T, Size>& v, bool& fully_in_volume)const
{
	return v.CollideWith(*this, fully_in_volume);
}
template<class T, int Size>
bool TAABB<T, Size>::CollideWith(const TFrustum<T, Size>& frustum)const
{
	return frustum.Overlaps(*this);
}
template<class T, int Size>
bool TAABB<T, Size>::CollideWith(const TFrustum<T, Size>& frustum, bool& fully_in_frustum)const
{
	return frustum.Overlaps(*this, fully_in_frustum);
}
template<class T, int Size>
bool TAABB<T, Size>::CollideWith(const TAABB<T, Size>& v)const
{
	return Collide<T, Size>(*this, v);
}
template<class T, int Size>
bool TAABB<T, Size>::CollideWith(const TAABB<T, Size>& v, bool& fully_in_aabb)const
{
	return Collide<T, Size>(*this, v, fully_in_aabb);
}
template<class T, int Size>
bool TAABB<T, Size>::CollideWith(const TOBB<T, Size>& v)const
{
	return Collide<T, Size>(v, *this);
}
template<class T, int Size>
bool TAABB<T, Size>::CollideWith(const TOBB<T, Size>& v, bool& fully_in_obb)const
{
	return Collide<T, Size>(v, *this, fully_in_obb);
}
template<class T, int Size>
bool TAABB<T, Size>::CollideWith(const TCapsule<T, Size>& v)const
{
	return Collide(v, *this);
}
template<class T, int Size>
bool TAABB<T, Size>::CollideWith(const TCapsule<T, Size>& v, bool& fully_in_capsule)const
{
	return Collide(v, *this, fully_in_capsule);
}
template<class T, int Size>
bool TAABB<T, Size>::CollideWith(const TSphere<T, Size>& v)const
{
	return Collide<T, Size>(*this, v);
}
template<class T, int Size>
bool TAABB<T, Size>::CollideWith(const TSphere<T, Size>& v, bool& fully_in_sphere)const
{
	return Collide<T, Size>(*this, v, fully_in_sphere);
}

template class TAABB<float,2>;
template class TAABB<float,3>;
template class TAABB<double, 2>;
template class TAABB<double, 3>;