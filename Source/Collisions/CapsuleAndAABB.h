#pragma once

template<class T, bool check_capsule_fully_in_aabb>
bool CapsuleAABBCollideSpecialized(const TAABB<T, 2>& aabb, const TCapsule<T, 2> &capsule, bool& capsule_fully_in_aabb)
{
	TVec<T, 2> a = aabb.GetHalfSize(),
		b_dir = capsule.segment.GetDir(),
		b = ((capsule.segment.p1 - capsule.segment.p0)*0.5).GetAbs(),
		pos_a = aabb.GetCenter(),
		pos_b = (capsule.segment.p1 + capsule.segment.p0)*0.5;

	TVec<T, 2> TT((pos_b - pos_a));
	T ra, rb, t;
	T min_distance = std::numeric_limits<T>().max();

	if (check_capsule_fully_in_aabb)
		capsule_fully_in_aabb = true;

	//проекции на оси AABB (т.е. X Y Z)
	for (int i = 0; i<2; i++)
	{
		//для AABB проекцией будет его размер по соответсвующей оси
		ra = a[i];
		//для отрезка проекцией будет его размер по соответсвующей оси
		rb = b[i];
		t = std::abs(TT[i]);
		if (t > ra + rb + capsule.radius)
			return false;
		T curr_dist = t - ra - rb;
		if (curr_dist < min_distance)
			min_distance = curr_dist;
		if (check_capsule_fully_in_aabb)
			capsule_fully_in_aabb = capsule_fully_in_aabb && (t < ra - rb - capsule.radius);
	}

	{
		{
			TVec<T, 2> proj = b_dir.Cross();
			ra = a.AbsScalarMul(proj);
			t = abs(TT*proj);
			if (t > ra + capsule.radius)
				return false;
			T curr_dist = t - ra;
			if (curr_dist < min_distance)
				min_distance = curr_dist;
		}
	}

	//проекция на направление отрезка
	{
		ra = a.AbsScalarMul(b_dir);
		rb = b.Length();
		t = abs(TT * b_dir);
		if (t > ra + rb + capsule.radius)
			return false;
		T curr_dist = t - ra - rb;
		if (curr_dist < min_distance)
			min_distance = curr_dist;
	}

	//расстояение от AABB до вершин отрезка
	TPointCollisionInfo<T, 2> info0, info1;
	aabb.PointCollide(capsule.segment.p0, info0);
	aabb.PointCollide(capsule.segment.p1, info1);

	if (info0.distance < min_distance)
		min_distance = info0.distance;

	if (info1.distance < min_distance)
		min_distance = info1.distance;

	return min_distance<capsule.radius;
}

template<class T, bool check_capsule_fully_in_aabb>
bool CapsuleAABBCollideSpecialized(const TAABB<T, 3>& aabb, const TCapsule<T, 3> &capsule, bool& capsule_fully_in_aabb)
{
	TVec<T, 3> a = aabb.GetHalfSize(),
		b_dir = capsule.segment.GetDir(),
		b = ((capsule.segment.p1 - capsule.segment.p0)*0.5).GetAbs(),
		pos_a = aabb.GetCenter(),
		pos_b = (capsule.segment.p1 + capsule.segment.p0)*0.5;

	TVec<T, 3> TT((pos_b - pos_a));
	T ra, rb, t;
	T min_distance = std::numeric_limits<T>().max();

	if (check_capsule_fully_in_aabb)
		capsule_fully_in_aabb = true;

	//проекции на оси AABB (т.е. X Y Z)
	for (int i = 0; i<3; i++)
	{
		//для AABB проекцией будет его размер по соответсвующей оси
		ra = a[i];
		//для отрезка проекцией будет его размер по соответсвующей оси
		rb = b[i];
		t = abs(TT[i]);
		if (t > ra + rb + capsule.radius)
			return false;
		T curr_dist = t - ra - rb;
		if (curr_dist < min_distance)
			min_distance = curr_dist;
		if (check_capsule_fully_in_aabb)
			capsule_fully_in_aabb = capsule_fully_in_aabb && (t < ra - rb - capsule.radius);
	}

	//проекции на 3 векторных произведения осей AABB и направления отрезка
	for (int i = 0; i<3; i++)
	{
		TVec<T, 3> temp(0);
		temp[i] = 1;
		{
			TVec<T, 3> proj = temp.Cross(b_dir);
			ra = a.AbsScalarMul(proj);
			t = abs(TT*proj);
			if (t > ra + capsule.radius)
				return false;
			T curr_dist = t - ra;
			if (curr_dist < min_distance)
				min_distance = curr_dist;
		}
	}

	//проекция на направление отрезка
	{
		ra = a.AbsScalarMul(b_dir);
		rb = b.Length();
		t = abs(TT * b_dir);
		if (t > ra + rb + capsule.radius)
			return false;
		T curr_dist = t - ra - rb;
		if (curr_dist < min_distance)
			min_distance = curr_dist;
	}
	
	//расстояение от AABB до вершин отрезка
	TPointCollisionInfo<T, 3> info0, info1;
	aabb.PointCollide(capsule.segment.p0, info0);
	aabb.PointCollide(capsule.segment.p1, info1);

	if (info0.distance < min_distance)
		min_distance = info0.distance;

	if (info1.distance < min_distance)
		min_distance = info1.distance;

	return min_distance<capsule.radius;
}

template<class T, int Size>
bool Collide(const TCapsule<T,Size>& capsule,const TAABB<T,Size>& aabb)
{
	bool not_used;
	return CapsuleAABBCollideSpecialized<T,false>(aabb, capsule, not_used);
}

template<class T, int Size>
bool Collide(const TCapsule<T, Size>& capsule, const TAABB<T, Size>& aabb, bool& capsule_fully_in_aabb)
{
	return CapsuleAABBCollideSpecialized<T, true>(aabb, capsule, capsule_fully_in_aabb);
}

template<class T>
bool AABBCapsuleCollideSpecialized(const TAABB<T, 2>& aabb, const TCapsule<T, 2> &capsule, bool& aabb_fully_in_capsule)
{
	aabb_fully_in_capsule = true;
	bool result = false;
	for (int i = 3; i >= 0; i--)
	{
		TVec<T, 2> aabb_vertex(aabb.border[(i >> 1) & 1][0], aabb.border[i & 1][1]);
		TVec<T, 2> nearest_point;
		T t;
		if (DistanceBetweenPointSegment(aabb_vertex, capsule.segment, t, nearest_point) < capsule.radius)
		{
			aabb_fully_in_capsule = aabb_fully_in_capsule && true;
			result = true;
		}
	}

	if (!result)
		return false;

	bool not_used;
	return CapsuleAABBCollideSpecialized<T, false>(aabb, capsule, not_used);
}

template<class T>
bool AABBCapsuleCollideSpecialized(const TAABB<T, 3>& aabb, const TCapsule<T, 3> &capsule, bool& aabb_fully_in_capsule)
{
	aabb_fully_in_capsule = true;
	bool result = false;
	for (int i = 7; i >= 0; i--)
	{
		TVec<T, 3> aabb_vertex(aabb.border[(i >> 2) & 1][0], aabb.border[(i >> 1) & 1][1], aabb.border[i & 1][2]);
		TVec<T, 3> nearest_point;
		T t;
		if (DistanceBetweenPointSegment(aabb_vertex, capsule.segment, t, nearest_point) < capsule.radius)
		{
			aabb_fully_in_capsule = aabb_fully_in_capsule && true;
			result = true;
		}
	}

	if (!result)
		return false;

	bool not_used;
	return CapsuleAABBCollideSpecialized<T, false>(aabb, capsule, not_used);
}

template<class T, int Size>
bool Collide(const TAABB<T, Size>& aabb, const TCapsule<T, Size>& capsule, bool& aabb_fully_in_capsule)
{
	return AABBCapsuleCollideSpecialized(aabb, capsule, aabb_fully_in_capsule);
}

