#include "common.h"

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
using namespace BaluLib;

class BaluLib::TTimePrivate
{
public:
	double dT, curr_fps;
	int frames;
	unsigned long long prev_time, freq, curr_time;
	double fps;
};

TTime::~TTime()
{
	delete p;
}

BaluLib::TTime::TTime()
{
	p = new TTimePrivate();
}

BaluLib::TTime::TTime(const TTime &source)
{
	p = new TTimePrivate(*source.p);
}

void BaluLib::TTime::operator=(const TTime &source)
{
	*p = *source.p;
}

unsigned long long TTime::GetTime()
{
#ifdef WIN32
	LARGE_INTEGER s;
	QueryPerformanceCounter(&s);
	return s.QuadPart;
#else
	timeval tv;
	gettimeofday(&tv, NULL);
	return (int)(tv.tv_sec * 1000 + (tv.tv_usec / 1000));
#endif
}

double TTime::TimeDiff(unsigned long long curTime, unsigned long long prevTime)
{
#ifdef WIN32
	return double(((double)(signed __int64)(curTime - prevTime)) / ((double)(signed __int64)p->freq));
#else
	return (curTime - prevTime)*0.001;
#endif
}

void TTime::Start() {
#ifdef WIN32
	LARGE_INTEGER s;
	QueryPerformanceFrequency(&s);
	p->freq = s.QuadPart;
#endif
	p->prev_time = GetTime();
	p->curr_fps = 0;
	p->fps = 0;
	p->frames = 0;
}
double TTime::GetDelta()
{
	return TimeDiff(GetTime(), p->prev_time);
}
void TTime::Tick() {
	p->curr_time = GetTime();
	p->dT = TimeDiff(p->curr_time, p->prev_time);
	p->prev_time = p->curr_time;
	p->frames++;
	p->curr_fps += p->dT;
	if (p->curr_fps > 0.5f) {
		p->fps = p->frames / p->curr_fps;
		p->curr_fps = 0;
		p->frames = 0;
	}
}
bool TTime::ShowFPS() {
	return !p->frames;
}
double TTime::GetTick() {
	return p->dT;
}
double TTime::GetFPS() {
	return p->fps;
}