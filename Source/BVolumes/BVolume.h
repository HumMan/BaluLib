template<class T,int Size>
class TBVolume
{
public:

	virtual bool Contain(const TVec<T,Size>& point) const=0;
	virtual bool Contain(const TVec<T,Size>& point,T& distance, TVec<T,Size>& nearest_point, TVec<T,Size>& normal) const=0;
	virtual bool CollideWith(const TRay<T,Size> &ray) const=0;
	virtual bool CollideWith(const TRay<T,Size> &ray, T& t, TVec<T,Size>& normal) const=0;
	virtual bool CollideWith(const TRay<T,Size> &ray, T& t0,T& t1) const=0;
	virtual bool CollideWith(const TRay<T,Size> &ray, T& t0, TVec<T,Size>& normal0,T& t1, TVec<T,Size>& normal1) const=0;

	virtual void DrawTriangles(std::vector<TVec<T, Size> >& vertices, std::vector<unsigned int>& indices) const = 0;
	virtual void DrawLines(std::vector<TVec<T, Size> >& vertices) const = 0;

	//
	virtual bool CollideWith(const TBVolume<T,Size>& v) const=0;
	virtual bool CollideWith(const TFrustum<T,Size>& frustum) const=0;
	virtual bool CollideWith(const TFrustum<T,Size>& frustum,bool& full_in_frustum) const=0;
	virtual bool CollideWith(const TAABB<T,Size>& v) const=0;
	virtual bool CollideWith(const TOBB<T,Size>& v) const=0;
	virtual bool CollideWith(const TCapsule<T,Size>& v) const=0;
	virtual bool CollideWith(const TSphere<T,Size>& v) const=0;

};
