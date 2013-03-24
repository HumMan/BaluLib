#pragma once

template<class T,int f_bits=8,bool precise=false>
class TFixedFloat
{
private:
	struct RAW
	{
		T v;
		RAW(T val){
			v=val;
		}
	};
	T v;
	TFixedFloat(RAW i){
		v=i.v;
	}
public:
	T GerRAW(){
		return v;
	}
	TFixedFloat(){}
	TFixedFloat(T i){
		v=i<<f_bits;
	}
	TFixedFloat(float f)
	{
		v=T(f*(1<<f_bits));
	}
	operator float()
	{
		return float(v)/(1<<f_bits);
	}
	operator int()
	{
		return v>>f_bits;
	}
	TFixedFloat operator<<(int i)
	{
		return TFixedFloat(RAW(v<<i));
	}
	TFixedFloat operator>>(int i)
	{
		return TFixedFloat(RAW(v>>i));
	}
	void operator<<=(int i)
	{
		v<<=i;
	}
	void operator>>=(int i)
	{
		v>>=i;
	}
	friend TFixedFloat operator+(TFixedFloat v1,TFixedFloat v2)//TODO везде должно быть TFIxedFloat<f_bits>
	{
		return TFixedFloat(RAW(v1.v+v2.v));
	}
	friend TFixedFloat operator-(TFixedFloat v1,TFixedFloat v2)
	{
		return TFixedFloat(RAW(v1.v-v2.v));
	}
	friend TFixedFloat operator/(TFixedFloat v1,TFixedFloat v2)
	{
		long long t1=v1.v;
		long long t2=v2.v;
		return TFixedFloat(RAW (((t1<<(f_bits*2)) /t2 )>>f_bits));
	}
	friend TFixedFloat operator*(TFixedFloat v1,TFixedFloat v2)
	{
		if(precise)
			return TFixedFloat(RAW(((long long)v1.v*(long long)v2.v)>>f_bits));
		else
			return TFixedFloat(RAW((v1.v*v2.v)>>f_bits));
	}

	void operator+=(TFixedFloat v2)
	{
		v+=v2.v;
	}
	void operator-=(TFixedFloat v2)
	{
		v-=v2.v;
	}
	void operator/=(TFixedFloat v2)
	{
		*this=*this/v2;
	}
	void operator*=(TFixedFloat v2)
	{
		*this=*this*v2;
	}

	friend bool operator<(TFixedFloat v1,TFixedFloat v2)
	{
		return v1.v<v2.v;
	}
	friend bool operator<=(TFixedFloat v1,TFixedFloat v2)
	{
		return v1.v<=v2.v;
	}
	friend bool operator>(TFixedFloat v1,TFixedFloat v2)
	{
		return v1.v>v2.v;
	}
	friend bool operator>=(TFixedFloat v1,TFixedFloat v2)
	{
		return v1.v>=v2.v;
	}
	friend bool operator==(TFixedFloat v1,TFixedFloat v2)
	{
		return v1.v==v2.v;
	}
};
