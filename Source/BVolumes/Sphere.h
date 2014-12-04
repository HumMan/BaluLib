#pragma once

template<class T,int Size>
class TSphere:public TBVolume<T,Size>
{
public:
	TVec<T, Size> pos;
	T radius;

	TSphere(){}
	TSphere(const TVec<T,Size>& use_pos,T use_radius):pos(use_pos),radius(use_radius){}
	TSphere(T use_radius):radius(use_radius)				{}
	void SetPos(const TVec<T,Size>& use_pos)				{pos=use_pos;}
	void SetRadius(T use_rad)								{radius=use_rad;}
	TAABB<T,Size> GetAABB()const							{return TAABB<T,Size>(pos,TVec3(radius*2));}
	T GetRadius()											{return radius;}
	TVec<T,Size> GetPos()									{return pos;}

	//Common virtual methods
	virtual bool Contain(const TVec<T,Size>& point) const;
	virtual bool Contain(const TVec<T,Size>& point,T& distance, TVec<T,Size>& nearest_point, TVec<T,Size>& normal) const;
	virtual bool CollideWith(const TRay<T,Size> &ray) const;
	virtual bool CollideWith(const TRay<T,Size> &ray, T& t, TVec<T,Size>& normal) const;
	virtual bool CollideWith(const TRay<T,Size> &ray, T& t0,T& t1) const;
	virtual bool CollideWith(const TRay<T,Size> &ray, T& t0, TVec<T,Size>& normal0,T& t1, TVec<T,Size>& normal1) const;

	virtual void DrawTriangles(std::vector<TVec<T, Size> >& vertices, std::vector<unsigned int>& indices) const;
	virtual void DrawLines(std::vector<TVec<T, Size> >& vertices) const;

	//
	virtual bool CollideWith(const TBVolume<T,Size>& v)const							{return v.CollideWith(*this);}
	virtual bool CollideWith(const TFrustum<T,Size>& frustum)const						{return frustum.Overlaps(*this);}
	virtual bool CollideWith(const TFrustum<T,Size>& frustum,bool& full_in_frustum)const{return frustum.Overlaps(*this,full_in_frustum);}
	virtual bool CollideWith(const TAABB<T,Size>& v)const								{return Collide<T,Size>(v,*this);}
	virtual bool CollideWith(const TOBB<T,Size>& v)const								{return Collide<T,Size>(*this,v);}
	virtual bool CollideWith(const TCapsule<T,Size>& v) const							{return Collide<T,Size>(v,*this);}
	virtual bool CollideWith(const TSphere<T,Size>& v) const							{return Collide<T,Size>(*this,v);}
};

struct TTri
{
	int rib[3];
	bool inv_dir[3];
	TTri(){}
	TTri(int r0,int r1,int r2,bool i0,bool i1,bool i2)
	{
		rib[0]=r0;
		rib[1]=r1;
		rib[2]=r2;
		inv_dir[0]=i0;
		inv_dir[1]=i1;
		inv_dir[2]=i2;
	}
};

template<class T,int Size>
void Tesselate(std::vector<TVec<T, Size> >& vertices, std::vector<TVec2ui>& ribs, std::vector<TTri>& triangles)
{
	int ribs_high=ribs.size()-1;
	ribs.resize(ribs.size()*2);
	for(int i=ribs_high;i>=0;i--)
	{
		TVec2ui rib=ribs[i];
		TVec<T,Size> middle=(vertices[rib[0]]+vertices[rib[1]])*0.5;
		vertices.push_back(middle);
		ribs[i*2+0]=TVec2ui(rib[0],vertices.size()-1);
		ribs[i*2+1]=TVec2ui(vertices.size()-1,rib[1]);
	}

	int triangles_high = triangles.size()-1;
	triangles.resize(triangles.size()*4);
	for(int i=triangles_high;i>=0;i--)
	{
		TTri tri=triangles[i];

		//����� ������������ ����������
		ribs.push_back(TVec2ui(ribs[tri.rib[0]*2+0][1],ribs[tri.rib[1]*2+0][1]));
		ribs.push_back(TVec2ui(ribs[tri.rib[1]*2+0][1],ribs[tri.rib[2]*2+0][1]));
		ribs.push_back(TVec2ui(ribs[tri.rib[2]*2+0][1],ribs[tri.rib[0]*2+0][1]));

		triangles[i*4+0]=TTri(
			tri.rib[0]*2+tri.inv_dir[0],
			ribs.size()-1-0,
			tri.rib[2]*2+!tri.inv_dir[2],
			tri.inv_dir[0],1,tri.inv_dir[2]);
		triangles[i*4+1]=TTri(
			tri.rib[0]*2+!tri.inv_dir[0],
			tri.rib[1]*2+tri.inv_dir[1],
			ribs.size()-1-2,
			tri.inv_dir[0],tri.inv_dir[1],1);
		triangles[i*4+2]=TTri(
			tri.rib[1]*2+!tri.inv_dir[1],
			tri.rib[2]*2+tri.inv_dir[2],
			ribs.size()-1-1,
			tri.inv_dir[1],tri.inv_dir[2],1);
		triangles[i*4+3]=TTri(
			ribs.size()-1-2,
			ribs.size() - 1 - 1,
			ribs.size() - 1 - 0,
			0,0,0);
	}
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

template<class T,int Size>
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
		vertices.push_back(TVec<T, 2>(float(radius*sin(i*step*M_PI / 180)), float(radius*cos(i*step*M_PI / 180))) + pos);
		vertices.push_back(TVec<T, 2>(float(radius*sin((i + 1)*step*M_PI / 180)), float(radius*cos((i + 1)*step*M_PI / 180))) + pos);
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

template<class T,int Size>
void TSphere<T, Size>::DrawLines(std::vector<TVec<T, Size> >& vertices)const
{
	DrawLinesSpecialized(*this, vertices);
}
