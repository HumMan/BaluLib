#pragma once

class TTime
{
	double dT,curr_fps;
	int frames;
	unsigned long long prev_time,freq,curr_time;
	double fps;
public:
	unsigned long long GetTime();
	double TimeDiff(unsigned long long curTime,unsigned long long prevTime);
	void Start();
	double GetDelta();
	void Tick();
	bool ShowFPS();
	double GetTick();
	double GetFPS();
};

