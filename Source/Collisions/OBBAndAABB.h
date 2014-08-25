template<class T,int Size>
bool Collide(const TOBB<T,Size>& v0,const TAABB<T,Size>& v1)
{
	static_assert(Size >= 2 && Size <= 3, "only 2d 3d support");
	TVec<T,Size> a=v1.GetSize(),
		b=v0.local.GetSize(),
		pos_a=v1.GetCenter(),
		pos_b=v0.pos;

	T ra,rb,t;
	int i,k;

	TVec<T,Size> TT((pos_b-pos_a));
	TMatrix<T,Size> R(v0.orient);
	TMatrix<T,Size> Rt(R);
	Rt.Transpose();

	if(Size==2)
	{
		//projection on A axises
		for( i=0 ; i<2 ; i++ )
		{
			ra = a[i]; 
			rb = b[0]*abs(R[0][i]) + b[1]*abs(R[1][i]);
			t = abs( TT[i] );
			if ( t > ra + rb )
				return false;
		}

		//projection on B axises
		for( k=0 ; k<2 ; k++ )
		{
			ra = a[0]*abs(R[k][0]) + a[1]*abs(R[k][1]);
			rb = b[k];
			t = abs( TT[0]*R[k][0] + TT[1]*R[k][1]);
			if ( t > ra + rb )
				return false;
		}
		return true;
	}
	if(Size==3)
	{
		for( i=0 ; i<3 ; i++ )
		{
			ra = a[i]; 
			rb = b[0]*abs(R[0][i]) + b[1]*abs(R[1][i]) + b[2]*abs(R[2][i]);
			t = abs( TT[i] );
			if ( t > ra + rb )
				return false;
		}

		for( k=0 ; k<3 ; k++ )
		{
			ra = a[0]*abs(R[k][0]) + a[1]*abs(R[k][1]) + a[2]*abs(R[k][2]);
			rb = b[k];
			t = abs( TT[0]*R[k][0] + TT[1]*R[k][1] + TT[2]*R[k][2] );
			if ( t > ra + rb )
				return false;
		}

		for(int i=0;i<3;i++)
		{
			TVec<T,Size> temp(0);
			temp[i]=1;
			for(int k=0;k<3;k++)
			{
				TVec<T,Size> proj=temp.Cross(R[k]);
				ra = a.AbsScalarMul(proj);
				rb = b.AbsScalarMul(Rt*proj);
				t = abs(TT*proj);
				if ( t > ra + rb )
					return false;
			}
		}
	}
}