// Test.cpp : Defines the entry point for the console application.
//

#include <baluLib.h>
#include <stdio.h>

using namespace BaluLib;

int main() {
	{
		TAllocator<int, 200> v;
		v.New();
		TMatrix4 m1, m2, m3;
		TFPSCamera cam(TVec3(1, 1, 1), TVec3(1, 1, 1), TVec3(0, 0, 1));
		TMatrix4 mmm = cam.GetView();
		m2=m1*m3;
		m1*=m2;
		TVec4 v1, v2, v3;
		v2=m1*v2;
		//
		m1*=m2;
		float det = m1.GetDet();
		m1*=m2;
		printf("%f",/*v2[0],v2[1],v2[2],*/det);
		m2 = m1.GetTransposed();
		v2 = m2 * v2;
		m1*=m2;
		printf("%f%f%f", v2[0], v2[1], v2[2]);
	}
	{
		TIndexedArray<float> arr(100);
		arr.Add(1);
		arr.Add(1);
	}

	return 0;
}

