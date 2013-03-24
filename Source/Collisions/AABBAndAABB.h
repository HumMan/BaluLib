template<class T,int Size>
bool Collide(const TAABB<T,Size>& v0,const TAABB<T,Size>& v1)
{
	for(int i=0;i<Size;i++)
		if(!Overlay(
			v0.border[0][i],
			v0.border[1][i],
			v1.border[0][i],
			v1.border[1][i]))
			return false;
	return true;
}

template<class T,int Size>
bool Collide(const TAABB<T,Size>& v0,const TAABB<T,Size>& v1, bool& v1_full_in_v0)
{
	v1_full_in_v0=true;
	for(int i=0;i<Size;i++)
	{
		bool t0=v0.border[0][i]<v1.border[1][i];
		if(!t0)return false;
		bool t1=v0.border[1][i]>v1.border[0][i];
		if(!t1)return false;
                v1_full_in_v0&=(v0.border[0][i]>v1.border[0][i]&&v0.border[1][i]<v1.border[1][i]);
	}
	return true;
}

template<class T,int Size>
bool Collide(const TAABB<T,Size>& v0,const TAABB<T,Size>& v1, T& distance)
{
	distance=0;
	bool result=true;
	for(int i=0;i<Size;i++)
	{
		if(v0.border[0][i]>v1.border[1][i])
		{
			result=false;
			distance+=Sqr(v0.border[0][i]-v1.border[1][i]);
		}
		else if(v0.border[1][i]<v1.border[0][i])
		{
			result=false;
			distance+=Sqr(v0.border[1][i]-v1.border[0][i]);
		}
	}
	distance=sqrt(distance);
	return result;
}
