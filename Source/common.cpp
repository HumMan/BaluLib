#include <baluLib.h>

#include "string.h"

#if defined(WIN32)||defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <X11/Xlib.h>
#include <X11/keysym.h>
#include <X11/keysymdef.h>
#include <sys/time.h>
#endif

unsigned long long TTime::GetTime()
{
#ifdef WIN32
	LARGE_INTEGER s;
	QueryPerformanceCounter(&s);
	return s.QuadPart;
#else
	timeval tv;
	gettimeofday(&tv,NULL);
	return (int)(tv.tv_sec*1000 + (tv.tv_usec / 1000));
#endif
}

double TTime::TimeDiff(unsigned long long curTime,unsigned long long prevTime)
{
#ifdef WIN32
	return double(((double)(signed __int64)(curTime-prevTime))/((double)(signed __int64)freq));
#else
	return (curTime-prevTime)*0.001;
#endif
}

void TTime::Start(){
#ifdef WIN32
	LARGE_INTEGER s;
	QueryPerformanceFrequency(&s);
	freq=s.QuadPart;
#endif
	prev_time=GetTime();
	curr_fps=0;
	fps=0;
	frames=0;
}
double TTime::GetDelta()
{
	return TimeDiff(GetTime(),prev_time);
}
void TTime::Tick(){
	curr_time=GetTime();
	dT=TimeDiff(curr_time,prev_time);
	prev_time=curr_time;
	frames++;
	curr_fps+=dT;
	if(curr_fps>0.5f){
		fps=frames/curr_fps;
		curr_fps=0;
		frames=0;
	}
}
bool TTime::ShowFPS(){
	return !frames;
}
double TTime::GetTick(){
	return dT;
}
double TTime::GetFPS(){
	return fps;
}