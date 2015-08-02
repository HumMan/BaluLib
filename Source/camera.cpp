#include "baluLib.h"

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

void TFPSCamera::KeyDown(Key key, float time, float mult)
{
	float s = speed * mult;
	switch (key)
	{
	case Key::Left:
		pos -= cam_orient[0] * s*time;
		break;
	case Key::Right:
		pos += cam_orient[0] * s*time;
		break;
	case Key::Up:
		pos += cam_orient[2] * s*time;
		break;
	case Key::Down:
		pos -= cam_orient[2] * s*time;
		break;
	}
}

void TFPSCamera::MouseMove(int x, int y)
{
	TVec2i mouse_pos;
	TQuaternion temp;

	mouse_pos = TVec2i(x, y);
	if(mouse_pos==middle_pos)return;

	TVec2 mouse_direction;
	mouse_direction=TVec2((float)(middle_pos[0]-mouse_pos[0]),(float)(middle_pos[1]-mouse_pos[1]))*mouse_sensitivity;

	pitch+=mouse_direction[1];
	yaw+=mouse_direction[0];
	UpdateView();
}
