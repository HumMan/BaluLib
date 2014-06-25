template<class T,int Size>
class TOBB:public TBVolume<T,Size>
{
	friend class TFrustum<T,Size>;

	friend bool Collide<T,Size>(const TOBB<T,Size>& v0,const TAABB<T,Size>& v1);
	friend bool Collide<T,Size>(const TOBB<T,Size>& v0,const TAABB<T,Size>& v1, bool& v1_fullin_v0);
	friend bool Collide<T,Size>(const TOBB<T,Size>& v0,const TOBB<T,Size>& v1);
	friend bool Collide<T,Size>(const TSphere<T,Size>& v0,const TOBB<T,Size>& v1);
private:
	TVec<T,Size> pos;
	TMatrix<T,Size> orient;
	TAABB<T,Size> local;
public:
	TOBB(){}
	TOBB(const TVec<T,Size>& use_pos, const TMatrix<T,Size>& use_orient, const TAABB<T,Size>& use_aabb)
		:pos(use_pos),orient(use_orient),local(use_aabb){}
	void SetPos(const TVec<T,Size>& use_pos)					{pos=use_pos;}
	void SetOrient(const TMatrix<T,Size>& use_orient)			{orient=use_orient;}
	TVec<T,Size> GetPos()const									{return pos;}
	TAABB<T,Size> GetAABB()const								{return TAABB<T,Size>(pos,orient.AbsMul(local.GetSize()));}

	//Common virtual methods
	virtual bool Contain(const TVec<T,Size>& point) const;
	virtual bool Contain(const TVec<T,Size>& point,T& distance, TVec<T,Size>& nearest_point, TVec<T,Size>& normal) const;
	virtual bool CollideWith(const TRay<T,Size> &ray) const;
	virtual bool CollideWith(const TRay<T,Size> &ray, T& t, TVec<T,Size>& normal) const;
	virtual bool CollideWith(const TRay<T,Size> &ray, T& t0,T& t1) const;
	virtual bool CollideWith(const TRay<T,Size> &ray, T& t0, TVec<T,Size>& normal0,T& t1, TVec<T,Size>& normal1) const;

	virtual void DrawTriangles(std::vector<TVec<T, Size> >& vertices, std::vector<unsigned int>& indices)const;
	virtual void DrawLines(std::vector<TVec<T, Size> >& vertices)const;
	
	virtual bool CollideWith(const TBVolume<T,Size>& v)const							{return v.CollideWith(*this);}
	virtual bool CollideWith(const TFrustum<T,Size>& frustum)const						{return frustum.Overlaps(*this);}
	virtual bool CollideWith(const TFrustum<T,Size>& frustum,bool& full_in_frustum)const{return frustum.Overlaps(*this,full_in_frustum);}
	virtual bool CollideWith(const TAABB<T,Size>& v)const								{return Collide<T,Size>(*this,v);}
	virtual bool CollideWith(const TOBB<T,Size>& v)const								{return Collide<T,Size>(v,*this);}
	virtual bool CollideWith(const TCapsule<T,Size>& v)const							{return Collide(v,*this);}
	virtual bool CollideWith(const TSphere<T,Size>& v)const								{return Collide<T,Size>(v,*this);}
};

template<class T,int Size>
void TOBB<T, Size>::DrawTriangles(std::vector<TVec<T, Size> >& vertices, std::vector<unsigned int>& indices)const
{
	int vertices_high=vertices.GetHigh()+1;
	local.DrawTriangles(vertices,indices);
	for(int i=vertices_high;i<=vertices.GetHigh();i++)
		vertices[i]=orient*vertices[i]+pos;
}

template<class T,int Size>
void TOBB<T, Size>::DrawLines(std::vector<TVec<T, Size> >& vertices)const
{
	int vertices_high=vertices.GetHigh()+1;
	local.DrawLines(vertices);
	for(int i=vertices_high;i<=vertices.GetHigh();i++)
		vertices[i]=orient*vertices[i]+pos;
}
