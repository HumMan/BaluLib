#pragma once

extern bool KeyDown(int key_char);

extern TVec2i GetCursorPos();
extern void SetCursorPos(TVec2i pos);

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


class TFileData
{
private:
	void* file;
	int fsize;
public:
	TFileData(const char* fname,const char* mode);
	~TFileData();
	void Close();
	char* ReadAll();
	int GetSize();
	void Read(void* v,int size)const;
	void Read(void* v,int el_size,int count)const;
	void Write(const void* v,int size);
	void Write(const void* v,int el_size,int count);
	void Read(int& v)const;
	void Read(bool& v)const;
	void Write(const int v)const;
	void Write(const bool v)const;
	void Write(const char* v)const;
};

