#include "baluLib.h"

#include <windows.h>

TFPSCamera::TFPSCamera(TVec3 use_pos,TVec3 use_dir,TVec3 use_up)
{
	active=false;
	speed=20;
	mouse_sensitivity=0.006f;
	yaw=0;
	pitch=0;//TODO dir,up
	//start_orient.SetIdentity();
	pos=use_pos;
	UpdateView();
}

void TFPSCamera::UpdateView()
{
	TVec3 dir_z,dir_x,dir;
	dir_z=TVec3(sin(yaw),0,cos(yaw));
	dir_x=TVec3(dir_z[2],0,-dir_z[0]);
	dir=dir_z.GetRotated(dir_x,pitch);

	TMatrix3 m(dir_x,dir.Cross(dir_x),dir);
	cam_orient=m;

		m.Transpose();
		view=TMatrix4(m,-(m*pos));
}

TMatrix4 TFPSCamera::GetView()
{
	return view;
}

void TFPSCamera::SetViewByKeyboard(float time)
{
	float s=(KeyDown(VK_SHIFT)?speed*3:speed);
	bool need_update=false;
	if(KeyDown('A')){
		need_update=true;
		pos-=cam_orient[0]*s*time;
	}
	if(KeyDown('D')){
		need_update=true;
		pos+=cam_orient[0]*s*time;
	}
	if(KeyDown('S')){
		need_update=true;
		pos+=cam_orient[2]*s*time;
	}
	if(KeyDown('W')){
		need_update=true;
		pos-=cam_orient[2]*s*time;
	}
	if(need_update)
		UpdateView();
}

void TFPSCamera::Update(float use_time)
{
	if (active)
	{
		SetViewByMouse();
		SetViewByKeyboard(use_time);
	}
}

void TFPSCamera::SetViewByMouse()
{
	TVec2i mouse_pos;
	TQuaternion temp;

	mouse_pos=GetCursorPos();
	if(mouse_pos==middle_pos)return;
	SetCursorPos(middle_pos);

	TVec2 mouse_direction;
	mouse_direction=TVec2((float)(middle_pos[0]-mouse_pos[0]),(float)(middle_pos[1]-mouse_pos[1]))*mouse_sensitivity;

	pitch+=mouse_direction[1];
	yaw+=mouse_direction[0];
	UpdateView();
}
