#pragma once

template<class T, bool check_fully_in>
bool CollideSpecialization(const TOBB<T, 2>& v0, const TAABB<T, 2>& v1, bool& obb_fully_in_aabb)
{
	TVec<T, 2> a = v1.GetSize(),
		b = v0.local.GetSize(),
		pos_a = v1.GetCenter(),
		pos_b = v0.pos;

	T ra, rb, t;
	int i, k;

	TVec<T, 2> TT((pos_b - pos_a));
	TMatrix<T, 2> R(v0.orient);
	TMatrix<T, 2> Rt(R);
	Rt.Transpose();

	bool result = true;
	obb_fully_in_aabb = true;

	//projection on A axises
	for (i = 0; i<2; i++)
	{
		ra = a[i];
		rb = b.AbsScalarMul(Rt[i]);
		t = abs(TT[i]);
		if (t > ra + rb)
		{
			if (check_fully_in)
				result = false;
			else return false;
		}
		if (check_fully_in)
			obb_fully_in_aabb = obb_fully_in_aabb && (t < ra - rb);
	}
	if (check_fully_in)
	{
		//для определения obb_fully_in_aabb достаточно проверить проекции на оси AABB
		if (!result)
			return false;
	}
	//projection on B axises
	for (k = 0; k<2; k++)
	{
		ra = a.AbsScalarMul(R[k]);
		rb = b[k];
		t = abs(TT* R[k]);
		if (t > ra + rb)
			return false;
	}
	return true;
}

template<class T, bool check_fully_in>
bool CollideSpecialization(const TOBB<T, 3>& v0, const TAABB<T, 3>& v1, bool& obb_fully_in_aabb)
{
	TVec<T, 3> a = v1.GetSize(),
		b = v0.local.GetSize(),
		pos_a = v1.GetCenter(),
		pos_b = v0.pos;

	T ra, rb, t;
	int i, k;

	TVec<T, 3> TT((pos_b - pos_a));
	TMatrix<T, 3> R(v0.orient);
	TMatrix<T, 3> Rt(R);
	Rt.Transpose();

	bool result = true;
	obb_fully_in_aabb = true;

	for (i = 0; i<3; i++)
	{
		ra = a[i];
		rb = b.AbsScalarMul(Rt[i]);
		t = abs(TT[i]);
		if (t > ra + rb)
		{
			if (check_fully_in)
				result = false;
			else return false;
		}
		if (check_fully_in)
			obb_fully_in_aabb = obb_fully_in_aabb && (t < ra - rb);
	}
	if (check_fully_in)
	{
		//для определения obb_fully_in_aabb достаточно проверить проекции на оси AABB
		if (!result)
			return false;
	}

	for (k = 0; k<3; k++)
	{
		ra = a.AbsScalarMul(R[k]);
		rb = b[k];
		t = abs(TT * R[k]);
		if (t > ra + rb)
			return false;
	}

	for (int i = 0; i<3; i++)
	{
		TVec<T, 3> temp(0);
		temp[i] = 1;
		for (int k = 0; k<3; k++)
		{
			TVec<T, 3> proj = temp.Cross(R[k]);
			ra = a.AbsScalarMul(proj);
			rb = b.AbsScalarMul(Rt*proj);
			t = abs(TT*proj);
			if (t > ra + rb)
				return false;
		}
	}

	return true;
}

template<class T,int Size>
bool Collide(const TOBB<T,Size>& v0,const TAABB<T,Size>& v1)
{
	static_assert(Size >= 2 && Size <= 3, "only 2d 3d support");
	bool not_used;
	return CollideSpecialization<T, false>(v0, v1, not_used);
}

template<class T, int Size>
bool Collide(const TOBB<T, Size>& v0, const TAABB<T, Size>& v1, bool& obb_fully_in_aabb)
{
	static_assert(Size >= 2 && Size <= 3, "only 2d 3d support");

	return CollideSpecialization<T, true>(v0, v1, obb_fully_in_aabb);
}

template<class T, int Size>
bool Collide(const TAABB<T, Size>& v1, const TOBB<T, Size>& v0, bool& abb_fully_in_obb)
{
	static_assert(Size >= 2 && Size <= 3, "only 2d 3d support");

	TAABB<T, Size> new_aabb(v0.local);
	TOBB<T, Size> new_obb(v0.GetPos(), v0.orient.GetTransposed(), v1);

	return CollideSpecialization<T, true>(new_obb, new_aabb, abb_fully_in_obb);
}