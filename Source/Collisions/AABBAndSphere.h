#pragma once

template<class T, int Size>
bool Collide(const TAABB<T, Size>& v0, const TSphere<T, Size>& v1)
{
	T sqr_distance;
	return Collide(v0, v1, sqr_distance);
}

template<class T, int Size>
bool Collide(const TAABB<T, Size>& v0, const TSphere<T, Size>& v1, bool& v1_fullin_v0)
{
	T sqr_distance = 0;
	v1_fullin_v0 = true;
	for (int i = 0; i < Size; i++)
	{
		if (v1.pos[i]<v0.border[0][i])
			sqr_distance += sqr(v0.border[0][i] - v1.pos[i]);
		else if (v1.pos[i]>v0.border[1][i])
			sqr_distance += sqr(v1.pos[i] - v0.border[1][i]);
		else if (v1.pos[i]<v0.border[0][i] + v1.radius || v1.pos[i]>v0.border[1][i] - v1.radius)
			v1_fullin_v0 = false;
	}
	return v1.radius*v1.radius > sqr_distance;
}

template<class T, int Size>
bool Collide(const TSphere<T, Size>& v0, const TAABB<T, Size>& v1, bool& v1_fullin_v0)
{
	T sqr_distance = 0;
	T fully_in_sqr_distance = 0;
	v1_fullin_v0 = true;
	for (int i = 0; i < Size; i++)
	{
		T t0 = sqr(v1.border[0][i] - v0.pos[i]);
		T t1 = sqr(v0.pos[i] - v1.border[1][i]);
		if (v0.pos[i] < v1.border[0][i])
			sqr_distance += t0;
		else if (v0.pos[i] > v1.border[1][i])
			sqr_distance += t1;
		fully_in_sqr_distance += std::fmax(t0, t1);
	}
	T sqr_radius = v0.radius*v0.radius;
	v1_fullin_v0 = sqr_radius > fully_in_sqr_distance;
	return sqr_radius > sqr_distance;
}

template<class T, int Size>
bool Collide(const TAABB<T, Size>& v0, const TSphere<T, Size>& v1, T& sqr_distance)
{
	sqr_distance = 0;
	for (int i = 0; i < Size; i++)
	{
		if (v1.pos[i]<v0.border[0][i])
			sqr_distance += sqr(v0.border[0][i] - v1.pos[i]);
		else if (v1.pos[i]>v0.border[1][i])
			sqr_distance += sqr(v1.pos[i] - v0.border[1][i]);
	}
	return v1.radius*v1.radius > sqr_distance;
}

