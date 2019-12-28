#pragma once
namespace BaluLib
{

	template<class T>
	class TQuaternion_base
	{
	private:
		T x, y, z, w;
	public:
		TQuaternion_base(){};
		TQuaternion_base(T use_w, T use_x, T use_y, T use_z)
		{
			x = use_x;
			y = use_y;
			z = use_z;
			w = use_w;
		};
		TQuaternion_base(T use_w, const TVec<T, 3>& v)
		{
			x = v[0];
			y = v[1];
			z = v[2];
			w = use_w;
		};
		TVec<T, 3> GetVector()
		{
			return TVec<T, 3>(x, y, z);
		};
		T GetW()
		{
			return w;
		};
		T Length()const
		{
			return sqrt(x*x + y*y + z*z + w*w);
		};
		void Normalize()
		{
			T l = (T)(1) / Length();
			w *= l;
			x *= l;
			y *= l;
			z *= l;
		};
		void Conjugate()
		{
			T t = (T)(1) / (x*x + y*y + z*z + w*w);
			x *= -t;
			y *= -t;
			z *= -t;
			w *= t;
		};
		TQuaternion_base GetNormalized()const
		{
			T l = (T)(1) / Length();
			return TQuaternion_base(w*l, x*l, y*l, z*l);
		};
		TQuaternion_base GetConjugated()const
		{
			T t = (T)(1) / (x*x + y*y + z*z + w*w);
			return TQuaternion_base(w*t, -x*t, -y*t, -z*t);
		};
		TQuaternion_base operator*(const TQuaternion_base& q)const
		{
			return TQuaternion_base(
				w*q.w - x*q.x - y*q.y - z*q.z,
				w*q.x + x*q.w + y*q.z - z*q.y,
				w*q.y - x*q.z + y*q.w + z*q.x,
				w*q.z + x*q.y - y*q.x + z*q.w);
		};
		void operator=(const TQuaternion_base& q)
		{
			x = q.x;
			y = q.y;
			z = q.z;
			w = q.w;
		};
		TQuaternion_base GetRotation(T use_angle, const TVec3& use_axis)
		{
			T t = sin(use_angle*0.5);
			return TQuaternion_base(cos(use_angle*0.5), use_axis*t);
		};
		TMatrix<T, 3> GetMatrix()
		{
			T
				x2 = 2 * x*x,
				y2 = 2 * y*y,
				z2 = 2 * z*z,
				xy = 2 * x*y,
				xz = 2 * x*z,
				wx = 2 * w*x,
				wy = 2 * w*y,
				wz = 2 * w*z,
				yz = 2 * y*z;
			return TMatrix<T, 3>(
				1 - y2 - z2, xy - wz, xz + wy,
				xy + wz, 1 - x2 - z2, yz - wx,
				xz - wy, yz + wx, 1 - x2 - y2);
		};
	};

	typedef TQuaternion_base<float> TQuaternion;
}