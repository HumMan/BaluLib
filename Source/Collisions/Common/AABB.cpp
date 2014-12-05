#include "../../BVolumes/AABB.h"

template<class T,int Size>
bool TAABB<T,Size>::Contain(const TVec<T,Size>& point) const
{
	for(int i=0;i<Size;i++)
	{
		if(!IsInMinMax(point[i],border[0][i],border[1][i]))
			return false;
	}
	return true;
}

template<class T,int Size>
bool TAABB<T,Size>::Contain(const TVec<T,Size>& point,T& distance, TVec<T,Size>& nearest_point, TVec<T,Size>& normal) const
{
	bool result=true;
	for(int i=0;i<Size;i++)
	{
		if(point[i]>border[1][i])
		{
			nearest_point[i]=border[1][i];
			result=false;
		}else if(point[i]<border[0][i])
		{
			nearest_point[i]=border[0][i];
			result=false;
		}
	}
	normal=nearest_point-point;
	normal.Normalize();
	return result;
}

template<class T, int Size>
bool TAABB<T, Size>::CollideWith(const TBVolume<T, Size>& v)const							{ return v.CollideWith(*this); }
template<class T, int Size>
bool TAABB<T, Size>::CollideWith(const TFrustum<T, Size>& frustum)const						{ return frustum.Overlaps(*this); }
template<class T, int Size>
bool TAABB<T, Size>::CollideWith(const TFrustum<T, Size>& frustum, bool& full_in_frustum)const{ return frustum.Overlaps(*this, full_in_frustum); }
template<class T, int Size>
bool TAABB<T, Size>::CollideWith(const TAABB<T, Size>& v)const								{ return Collide<T, Size>(*this, v); }
template<class T, int Size>
bool TAABB<T, Size>::CollideWith(const TOBB<T, Size>& v)const								{ return Collide<T, Size>(v, *this); }
template<class T, int Size>
bool TAABB<T, Size>::CollideWith(const TCapsule<T, Size>& v)const							{ return Collide(v, *this); }
template<class T, int Size>
bool TAABB<T, Size>::CollideWith(const TSphere<T, Size>& v)const								{ return Collide<T, Size>(*this, v); }

template<class T,int Size>
bool TAABB<T,Size>::CollideWith(const TRay<T,Size> &ray) const
{
	static_assert(Size >= 2 && Size <= 3, "only 2d 3d support");
	if(Size==2)
	{
		if(ray.dir[0]>0){
			if(ray.pos[0]>border[1][0])return false;
		}else if(ray.pos[0]<border[0][0])return false;
		if(ray.dir[1]>0){
			if(ray.pos[1]>border[1][1])return false;
		}else if(ray.pos[1]<border[0][1])return false;
		TVec<T,Size>
			size=GetSize(),
			v=GetCenter()-ray.pos;
		if(abs(v[1]*ray.dir[0]-v[0]*ray.dir[1])>size[0]*abs(ray.dir[1])+size[1]*abs(ray.dir[0]))
			return false;
		return true;
	}
	if(Size==3)
	{
		if(ray.dir[0]>0){
			if(ray.pos[0]>border[1][0])return false;
		}else if(ray.pos[0]<border[0][0])return false;
		if(ray.dir[1]>0){
			if(ray.pos[1]>border[1][1])return false;
		}else if(ray.pos[1]<border[0][1])return false;
		if(ray.dir[2]>0){
			if(ray.pos[2]>border[1][2])return false;
		}else if(ray.pos[2]<border[0][2])return false;
		TVec<T,Size>
			size=GetSize(),
			v=GetCenter()-ray.pos;
		if(abs(v[2]*ray.dir[1]-v[1]*ray.dir[2])>size[1]*abs(ray.dir[2])+size[2]*abs(ray.dir[1]))
			return false;
		if(abs(v[0]*ray.dir[2]-v[2]*ray.dir[0])>size[0]*abs(ray.dir[2])+size[2]*abs(ray.dir[0]))
			return false;
		if(abs(v[0]*ray.dir[1]-v[1]*ray.dir[0])>size[0]*abs(ray.dir[1])+size[1]*abs(ray.dir[0]))
			return false;
		return true;
	}
	return true;
}

template<class T>
bool CollideAxis(T start,T first,T last,T dir,T& cf, T& cl,bool& f_curr_cut,bool& f_cut_min,bool& l_curr_cut,bool& l_cut_min)
{
	f_curr_cut=false;
	l_curr_cut=false;
	T f=(first-start)/dir;
	T l=(last-start)/dir;
	if(f<0&&l<0)return false;
	if(dir>=0)
	{
		if(f>cf)
		{
			f_curr_cut=true;
			f_cut_min=true;
			cf=f;
		}
		if(l<cl)
		{
			l_curr_cut=true;
			l_cut_min=false;
			cl=l;
		}
	}else
	{
		if(l>cf)
		{
			f_curr_cut=true;
			f_cut_min=false;
			cf=l;
		}
		if(f<cl)
		{
			l_curr_cut=true;
			l_cut_min=true;
			cl=f;
		}
	}
	if(cf>cl)return false;
	return true;
}

template<class T,int Size>
bool TAABB<T,Size>::CollideWith(const TRay<T,Size> &ray, T& t0,T& t1) const
{
	bool result=true;
	T cf=0,cl=10e30;
	int fcut=-1,lcut=-1;
	bool fmin=false,lmin=false;
	for(int i=0;i<Size&&result;i++)
	{
		bool f_curr_cut,f_cut_min;
		bool l_curr_cut,l_cut_min;
		result=result&&CollideAxis<T>(ray.pos[i],border[0][i],border[1][i],ray.dir[i],cf,cl,f_curr_cut,f_cut_min,l_curr_cut,l_cut_min);
		if(f_curr_cut)
		{
			fcut=i;
			fmin=f_cut_min;
		}
		if(l_curr_cut)
		{
			lcut=i;
			lmin=l_cut_min;
		}
	}
	//TODO обработка случая когда начало луча внутри объема
	if(result)assert(fcut!=-1||lcut!=-1);
	t0=cf;
	t1=cl;
	return result;
}

template<class T,int Size>
bool TAABB<T,Size>::CollideWith(const TRay<T,Size> &ray, T& t, TVec<T,Size>& normal) const
{
	bool result=true;
	T cf=0,cs=10e30;
	int fcut=-1,lcut=-1;
	bool fmin=false,lmin=false;
	for(int i=0;i<Size&&result;i++)
	{
		bool f_curr_cut,f_cut_min;
		bool l_curr_cut,l_cut_min;
		result=result&&CollideAxis<T>(ray.pos[i],border[0][i],border[1][i],ray.dir[i],cf,cs,f_curr_cut,f_cut_min,l_curr_cut,l_cut_min);
		if(f_curr_cut)
		{
			fcut=i;
			fmin=f_cut_min;
		}
		if(l_curr_cut)
		{
			lcut=i;
			lmin=l_cut_min;
		}
	}
	if(result)assert(fcut!=-1&&lcut!=-1);
	normal=TVec<T,Size>(0);
	if(result)
		if((fcut==-1?lcut:fcut)!=-1)
		normal[fcut==-1?lcut:fcut]=(fcut==-1?lmin:fmin)?-1:1;
	return result;
}

template<class T,int Size>
bool TAABB<T,Size>::CollideWith(const TRay<T,Size> &ray, T& t0, TVec<T,Size>& normal0,T& t1, TVec<T,Size>& normal1) const
{
	bool result=true;
	T cf=0,cl=10e30;
	int fcut=-1,lcut=-1;
	bool fmin=false,lmin=false;
	for(int i=0;i<Size&&result;i++)
	{
		bool f_curr_cut,f_cut_min;
		bool l_curr_cut,l_cut_min;
		result=result&&CollideAxis<T>(ray.pos[i],border[0][i],border[1][i],ray.dir[i],cf,cl,f_curr_cut,f_cut_min,l_curr_cut,l_cut_min);
		if(f_curr_cut)
		{
			fcut=i;
			fmin=f_cut_min;
		}
		if(l_curr_cut)
		{
			lcut=i;
			lmin=l_cut_min;
		}
	}
	if(result)assert(fcut!=-1&&lcut!=-1);
	
	if(result)
	{
		t0=cf;
		t1=cl;
		normal0=TVec<T,Size>(0);
		normal0[fcut]=fmin?-1:1;
		normal1=TVec<T,Size>(0);
		normal1[lcut]=lmin?-1:1;
	}
	return result;
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
	vertices.push_back(TVec<T, 2>(border[0][0], border[0][1]));
	vertices.push_back(TVec<T, 2>(border[1][0], border[0][1]));

	vertices.push_back(TVec<T, 2>(border[0][0], border[0][1]));
	vertices.push_back(TVec<T, 2>(border[0][0], border[1][1]));

	vertices.push_back(TVec<T, 2>(border[0][0], border[1][1]));
	vertices.push_back(TVec<T, 2>(border[1][0], border[1][1]));

	vertices.push_back(TVec<T, 2>(border[1][0], border[0][1]));
	vertices.push_back(TVec<T, 2>(border[1][0], border[1][1]));
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

template class TAABB<float,2>;
template class TAABB<float,3>;
template class TAABB<double, 2>;
template class TAABB<double, 3>;