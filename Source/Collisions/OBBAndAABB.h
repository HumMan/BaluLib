#pragma once

template<class T>
bool CollideSpecialization(const TOBB<T, 2>& v0, const TAABB<T, 2>& v1)
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

		//projection on A axises
		for (i = 0; i<2; i++)
		{
			ra = a[i];
			//rb = b[0] * abs(R[0][i]) + b[1] * abs(R[1][i]);
			rb = b.AbsScalarMul(Rt[i]);
			t = abs(TT[i]);
			if (t > ra + rb)
				return false;
		}

		//projection on B axises
		for (k = 0; k<2; k++)
		{
			//ra = a[0] * abs(R[k][0]) + a[1] * abs(R[k][1]);
			ra = a.AbsScalarMul(R[k]);
			rb = b[k];
			//t = abs(TT[0] * R[k][0] + TT[1] * R[k][1]);
			t = abs(TT* R[k]);
			if (t > ra + rb)
				return false;
		}
		return true;
}

template<class T>
bool CollideSpecialization(const TOBB<T, 3>& v0, const TAABB<T, 3>& v1)
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

		for (i = 0; i<3; i++)
		{
			ra = a[i];
			//rb = b[0] * abs(R[0][i]) + b[1] * abs(R[1][i]) + b[2] * abs(R[2][i]);
			rb = b.AbsScalarMul(Rt[i]);
			t = abs(TT[i]);
			if (t > ra + rb)
				return false;
		}

		for (k = 0; k<3; k++)
		{
			//ra = a[0] * abs(R[k][0]) + a[1] * abs(R[k][1]) + a[2] * abs(R[k][2]);
			ra = a.AbsScalarMul(R[k]);
			rb = b[k];
			//t = abs(TT[0] * R[k][0] + TT[1] * R[k][1] + TT[2] * R[k][2]);
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
}

template<class T,int Size>
bool Collide(const TOBB<T,Size>& v0,const TAABB<T,Size>& v1)
{
	static_assert(Size >= 2 && Size <= 3, "only 2d 3d support");

	CollideSpecialization(v0, v1);
}