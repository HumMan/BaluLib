
template<class T,int Size>
class TAABB:public TBVolume<T,Size>
{
	friend class TFrustum<T,Size>;
	friend class TOBB<T,Size>;

	friend bool Collide<T,Size>(const TOBB<T,Size>& v0,const TAABB<T,Size>& v1);
	friend bool Collide<T,Size>(const TAABB<T,Size>& v0,const TAABB<T,Size>& v1);
	friend bool Collide<T,Size>(const TAABB<T,Size>& v0,const TSphere<T,Size>& v1, T& sqr_distance);
private:
	TVec<T,Size> border[2]; //0-min 1-max
public:

	TAABB(){}
	TAABB(const TVec<T,Size>& use_pos,const TVec<T,Size>& use_widths)
	{
		border[0]=use_pos-use_widths;
		border[1]=use_pos+use_widths;
	}
	TVec<T,Size> GetSize()const									{return (border[1]-border[0])*0.5;}
	TVec<T,Size> GetPosition()const								{return GetCenter();}
	TVec<T,Size> GetCenter()const								{return (border[1]+border[0])*0.5;}
	void Set(int use_bound,int use_dim,T use_val)				{border[use_bound][use_dim]=use_val;}
	TVec<T,Size> operator[](int use_bound)const					{return border[use_bound];}

	void operator+=(const TVec<T,Size>& pos)
	{
		for(int k=0;k<Size;k++)
		{
			if     (pos[k]>border[1][k])border[1][k]=pos[k];
			else if(pos[k]<border[0][k])border[0][k]=pos[k];
		}
	}
	void Extend(TVec<T,Size> v)
	{
		TVec<T,Size> ext(v);
		border[0]-=ext;
		border[1]+=ext;
	}
	//generic for trees
	void ToSubCube(int i, int k) //i - x min max    k - y min max
	{
		COMPILE_TIME_ERR(Size==2);
		if(i==0)border[1][0]=(border[0][0]+border[1][0])*0.5f;
		else	border[0][0]=(border[0][0]+border[1][0])*0.5f;
		if(k==0)border[1][1]=(border[0][1]+border[1][1])*0.5f;
		else	border[0][1]=(border[0][1]+border[1][1])*0.5f;
	}
	void ToSubCube(int i, int k,int t)
	{
		COMPILE_TIME_ERR(Size==3);
		if(i==0)border[1][0]=(border[0][0]+border[1][0])*0.5f;
		else	border[0][0]=(border[0][0]+border[1][0])*0.5f;
		if(k==0)border[1][1]=(border[0][1]+border[1][1])*0.5f;
		else	border[0][1]=(border[0][1]+border[1][1])*0.5f;
		if(t==0)border[1][2]=(border[0][2]+border[1][2])*0.5f;
		else	border[0][2]=(border[0][2]+border[1][2])*0.5f;
	}
	void Draw(TBaluRender* render)const;

	//Common virtual methods
	virtual bool Contain(const TVec<T,Size>& point) const;
	virtual bool Contain(const TVec<T,Size>& point,T& distance, TVec<T,Size>& nearest_point, TVec<T,Size>& normal) const;
	virtual bool CollideWith(const TRay<T,Size> &ray) const;
	virtual bool CollideWith(const TRay<T,Size> &ray, T& t, TVec<T,Size>& normal) const;
	virtual bool CollideWith(const TRay<T,Size> &ray, T& t0,T& t1) const;
	virtual bool CollideWith(const TRay<T,Size> &ray, T& t0, TVec<T,Size>& normal0,T& t1, TVec<T,Size>& normal1) const;
        virtual void DrawTriangles(TVector<TVec<T,Size> >& vertices,TVector<unsigned int>& indices)const;
        virtual void DrawLines(TVector<TVec<T,Size> >& vertices)const;
	
	virtual bool CollideWith(const TBVolume<T,Size>& v)const							{return v.CollideWith(*this);}
	virtual bool CollideWith(const TFrustum<T,Size>& frustum)const						{return frustum.Overlaps(*this);}
	virtual bool CollideWith(const TFrustum<T,Size>& frustum,bool& full_in_frustum)const{return frustum.Overlaps(*this,full_in_frustum);}
	virtual bool CollideWith(const TAABB<T,Size>& v)const								{return Collide<T,Size>(*this,v);}
	virtual bool CollideWith(const TOBB<T,Size>& v)const								{return Collide<T,Size>(v,*this);}
	virtual bool CollideWith(const TCapsule<T,Size>& v)const							{return Collide(v,*this);}
	virtual bool CollideWith(const TSphere<T,Size>& v)const								{return Collide<T,Size>(*this,v);}
};

const bool quads_of_box[6][4][3]=
{
	{{0,0,0},	{0,0,1},	{0,1,1},	{0,1,0}},  //quad 1
	{{1,0,0},	{1,1,0},	{1,1,1},	{1,0,1}},  //quad 2
	{{0,0,0},	{1,0,0},	{1,0,1},	{0,0,1}},  //quad 3
	{{0,1,0},	{0,1,1},	{1,1,1},	{1,1,0}},  //quad 4
	{{0,0,0},	{0,1,0},	{1,1,0},	{1,0,0}},  //quad 5
	{{0,0,1},	{1,0,1},	{1,1,1},	{0,1,1}}   //quad 6
};

const bool tri_ribs_of_box[24][3]=
{
        {0,0,0}, {1,0,0}, {0,1,0}, {1,1,0}, {0,1,1}, {1,1,1}, {0,0,1}, {1,0,1},
        {0,0,0}, {0,1,0}, {1,0,0}, {1,1,0}, {1,0,1}, {1,1,1}, {0,0,1}, {0,1,1},
        {0,0,0}, {0,0,1}, {1,0,0}, {1,0,1}, {1,1,0}, {1,1,1}, {0,1,0}, {0,1,1},
};

template<class T,int Size>
void TAABB<T,Size>::DrawTriangles(TVector<TVec<T,Size> >& vertices,TVector<unsigned int>& indices)const
{
	COMPILE_TIME_ERR(Size>=2&&Size<=3);
	if(Size==2)
	{
		vertices.Inc(4);
		vertices.GetTop(3)=TVec<T,Size>(border[0][0],border[0][1]);
		vertices.GetTop(2)=TVec<T,Size>(border[0][0],border[1][1]);
		vertices.GetTop(1)=TVec<T,Size>(border[1][0],border[1][1]);
		vertices.GetTop(0)=TVec<T,Size>(border[1][0],border[0][1]);

		
		indices.Inc(6);
		indices.GetTop(5)=vertices.GetHigh()-3;
		indices.GetTop(4)=vertices.GetHigh()-2;
		indices.GetTop(3)=vertices.GetHigh()-1;

		indices.GetTop(2)=vertices.GetHigh()-3;
		indices.GetTop(1)=vertices.GetHigh()-0;
		indices.GetTop(0)=vertices.GetHigh()-1;
	}else if(Size==3)
	{
		int vertices_first=vertices.GetHigh()+1;
		vertices.Inc(8);
		for(int i=0;i<8;i++)
			vertices.GetTop(i)=TVec<T,Size>(border[(i>>2)&1][0],border[(i>>1)&1][1],border[i&1][2]);
		indices.Inc(6*3*2);
		for(int i=0;i<6;i++)
		{
			indices.GetTop(i*3*2+0)=vertices_first+(quads_of_box[i][0][2]<<2)+(quads_of_box[i][0][1]<<1)+quads_of_box[i][0][0];
			indices.GetTop(i*3*2+1)=vertices_first+(quads_of_box[i][1][2]<<2)+(quads_of_box[i][1][1]<<1)+quads_of_box[i][1][0];
			indices.GetTop(i*3*2+2)=vertices_first+(quads_of_box[i][2][2]<<2)+(quads_of_box[i][2][1]<<1)+quads_of_box[i][2][0];
																									  
			indices.GetTop(i*3*2+3)=vertices_first+(quads_of_box[i][0][2]<<2)+(quads_of_box[i][0][1]<<1)+quads_of_box[i][0][0];
			indices.GetTop(i*3*2+4)=vertices_first+(quads_of_box[i][3][2]<<2)+(quads_of_box[i][3][1]<<1)+quads_of_box[i][3][0];
			indices.GetTop(i*3*2+5)=vertices_first+(quads_of_box[i][2][2]<<2)+(quads_of_box[i][2][1]<<1)+quads_of_box[i][2][0];
		}
	}
}

template<class T,int Size>
void TAABB<T,Size>::DrawLines(TVector<TVec<T,Size> >& vertices)const
{
	COMPILE_TIME_ERR(Size>=2&&Size<=3);
	if(Size==2)
	{
		vertices.Inc(4*2);
		vertices.GetTop(7)=TVec<T,Size>(border[0][0],border[0][1]);
		vertices.GetTop(6)=TVec<T,Size>(border[1][0],border[0][1]);

		vertices.GetTop(5)=TVec<T,Size>(border[0][0],border[0][1]);
		vertices.GetTop(4)=TVec<T,Size>(border[0][0],border[1][1]);

		vertices.GetTop(3)=TVec<T,Size>(border[0][0],border[1][1]);
		vertices.GetTop(2)=TVec<T,Size>(border[1][0],border[1][1]);

		vertices.GetTop(1)=TVec<T,Size>(border[1][0],border[0][1]);
		vertices.GetTop(0)=TVec<T,Size>(border[1][0],border[1][1]);

	}else if(Size==3)
	{
		vertices.Inc(24);
		for(int i=24-1;i>=0;i--)
			vertices.GetTop(i)=TVec<T,Size>
			(
			border[tri_ribs_of_box[i][0]][0],
			border[tri_ribs_of_box[i][1]][1],
			border[tri_ribs_of_box[i][2]][2]
		);
	}
}
