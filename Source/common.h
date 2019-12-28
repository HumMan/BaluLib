#pragma once

#include "../Include/export.h"
#include <memory>
namespace BaluLib
{
	class TTimePrivate;
	class BALULIB_DLL_INTERFACE TTime
	{
		TTimePrivate* p;
	public:
		TTime();
		TTime(const TTime&);
		void operator=(const TTime&);
		~TTime();

		unsigned long long GetTime();
		double TimeDiff(unsigned long long curTime, unsigned long long prevTime);
		void Start();
		double GetDelta();
		void Tick();
		bool ShowFPS();
		double GetTick();
		double GetFPS();
	};

}