
template<class T,int Size>
class TSphere:public TBVolume<T,Size>
{
	friend class TFrustum<T,Size>;

	friend bool Collide<T,Size>(const TAABB<T,Size>& v0,const TSphere<T,Size>& v1, T& sqr_distance);
	friend bool Collide<T,Size>(const TSphere<T,Size>& v0,const TOBB<T,Size>& v1);
	friend bool Collide<T,Size>(const TSphere<T,Size>& v0,const TSphere<T,Size>& v1);
	friend bool Collide<T,Size>(const TAABB<T,Size>& v0,const TSphere<T,Size>& v1);
private:
	TVec<T,Size> pos;
	T radius;
public:
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
	int ribs_high=ribs.GetHigh();
	ribs.SetCount(ribs.GetCount()*2);
	for(int i=ribs_high;i>=0;i--)
	{
		TVec2ui rib=ribs[i];
		TVec<T,Size> middle=(vertices[rib[0]]+vertices[rib[1]])*0.5;
		vertices.Push(middle);
		ribs[i*2+0]=TVec2ui(rib[0],vertices.GetHigh());
		ribs[i*2+1]=TVec2ui(vertices.GetHigh(),rib[1]);
	}

	int triangles_high=triangles.GetHigh();
	triangles.SetCount(triangles.GetCount()*4);
	for(int i=triangles_high;i>=0;i--)
	{
		TTri tri=triangles[i];

		//ребра треугольника посередине
		ribs.Push(TVec2ui(ribs[tri.rib[0]*2+0][1],ribs[tri.rib[1]*2+0][1]));
		ribs.Push(TVec2ui(ribs[tri.rib[1]*2+0][1],ribs[tri.rib[2]*2+0][1]));
		ribs.Push(TVec2ui(ribs[tri.rib[2]*2+0][1],ribs[tri.rib[0]*2+0][1]));

		triangles[i*4+0]=TTri(
			tri.rib[0]*2+tri.inv_dir[0],
			ribs.GetHigh()-0,
			tri.rib[2]*2+!tri.inv_dir[2],
			tri.inv_dir[0],1,tri.inv_dir[2]);
		triangles[i*4+1]=TTri(
			tri.rib[0]*2+!tri.inv_dir[0],
			tri.rib[1]*2+tri.inv_dir[1],
			ribs.GetHigh()-2,
			tri.inv_dir[0],tri.inv_dir[1],1);
		triangles[i*4+2]=TTri(
			tri.rib[1]*2+!tri.inv_dir[1],
			tri.rib[2]*2+tri.inv_dir[2],
			ribs.GetHigh()-1,
			tri.inv_dir[1],tri.inv_dir[2],1);
		triangles[i*4+3]=TTri(
			ribs.GetHigh()-2,
			ribs.GetHigh()-1,
			ribs.GetHigh()-0,
			0,0,0);
	}
}


template<class T,int Size>
void TSphere<T, Size>::DrawTriangles(std::vector<TVec<T, Size> >& vertices, std::vector<unsigned int>& indices)const
{
	if(Size==2)
	{
		//TODO
	}
	else if(Size==3)
	{
		std::vector<TVec2ui> ribs;
		std::vector<TTri> triangles;

		int vertices_first=vertices.GetHigh()+1;
		vertices.Inc(6);
		vertices[vertices_first+0]=TVec<T,Size>(0, 0, radius);		//0
		vertices[vertices_first+1]=TVec<T,Size>(-radius, 0, 0);		//1
		vertices[vertices_first+2]=TVec<T,Size>(0, 0, -radius);		//2
		vertices[vertices_first+3]=TVec<T,Size>(radius, 0, 0);		//3
		vertices[vertices_first+4]=TVec<T,Size>(0, radius, 0);		//4
		vertices[vertices_first+5]=TVec<T,Size>(0, -radius, 0);		//5
		//
		ribs.SetHigh(11);
		ribs[0]=TVec2ui(0, 4);		//0
		ribs[1]=TVec2ui(1, 4);		//1
		ribs[2]=TVec2ui(2, 4);		//2
		ribs[3]=TVec2ui(3, 4);		//3
		ribs[4]=TVec2ui(0, 1);		//4
		ribs[5]=TVec2ui(1, 2);		//5
		ribs[6]=TVec2ui(2, 3);		//6
		ribs[7]=TVec2ui(3, 0);		//7
		ribs[8]=TVec2ui(0, 5);		//8
		ribs[9]=TVec2ui(1, 5);		//9
		ribs[10]=TVec2ui(2, 5);		//10
		ribs[11]=TVec2ui(3, 5);		//11
		for(int i=0;i<=11;i++)ribs[i]+=TVec2ui(vertices_first);
		//
		triangles.SetHigh(7);
		triangles[0]=TTri(0, 1, 4,	0,1,1);
		triangles[1]=TTri(7, 3, 0,	1,0,1);
		triangles[2]=TTri(8, 11, 7,	0,1,0);
		triangles[3]=TTri(4, 9, 8,	0,0,1);
		triangles[4]=TTri(5, 10, 9,	0,0,1);
		triangles[5]=TTri(2, 5, 1,	1,1,0);
		triangles[6]=TTri(6, 2, 3,	1,0,1);
		triangles[7]=TTri(10, 6, 11,1,0,0);

		for(int i=0;i<4;i++)
			Tesselate(vertices,ribs,triangles);

		for(int i=vertices_first;i<=vertices.GetHigh();i++)
			vertices[i]=pos+(vertices[i]).GetNormalized()*radius;

		int indices_first=indices.GetHigh()+1;
		indices.Inc(triangles.GetCount()*3);
		for(int i=0;i<=triangles.GetHigh();i++)
		{
			indices[indices_first+i*3+0]=ribs[triangles[i].rib[0]][triangles[i].inv_dir[0]];
			indices[indices_first+i*3+1]=ribs[triangles[i].rib[1]][triangles[i].inv_dir[1]];
			indices[indices_first+i*3+2]=ribs[triangles[i].rib[2]][triangles[i].inv_dir[2]];
		}

		
	}
}

template<class T,int Size>
void TSphere<T, Size>::DrawLines(std::vector<TVec<T, Size> >& vertices)const
{
	if(Size==2)
	{
		const int step=20;
		const int v_count=int(360/step)+1;
		for(int i=0;i<v_count;i++)
		{
			vertices.Push(TVec<T,Size>(float(radius*sin(i*step*M_PI/180)),float(radius*cos(i*step*M_PI/180)))+pos);
			vertices.Push(TVec<T,Size>(float(radius*sin((i+1)*step*M_PI/180)),float(radius*cos((i+1)*step*M_PI/180)))+pos);
		}
	}
	else if(Size==3)
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

		int v_count=20;
		T step=2*M_PI/v_count;
		T alpha0,alpha1;
		int vertices_last_high=vertices.GetHigh()+1;
		for(int i=0;i<v_count;i++)
		{
			alpha0=i*step;
			alpha1=alpha0+step;
			vertices.Push(TVec<T,Size>(radius*cos(alpha0),radius*sin(alpha0),0));
			vertices.Push(TVec<T,Size>(radius*cos(alpha1),radius*sin(alpha1),0));
		}
		for(int i=0;i<v_count;i++)
		{
			alpha0=i*step;
			alpha1=alpha0+step;
			vertices.Push(TVec<T,Size>(radius*cos(alpha0),0,radius*sin(alpha0)));
			vertices.Push(TVec<T,Size>(radius*cos(alpha1),0,radius*sin(alpha1)));
		}
		for(int i=0;i<v_count;i++)
		{
			alpha0=i*step;
			alpha1=alpha0+step;
			vertices.Push(TVec<T,Size>(0,radius*cos(alpha0),radius*sin(alpha0)));
			vertices.Push(TVec<T,Size>(0,radius*cos(alpha1),radius*sin(alpha1)));
		}
		for(int i=vertices_last_high;i<=vertices.GetHigh();i++)
			vertices[i]=pos+vertices[i];
	}
}
