#include "camera.h"

#include <math.h>

#include "Math/quaternion.h"

using namespace BaluLib;

class BaluLib::TFPSCameraPrivate
{
public:
	float yaw,
		pitch;
	//TMatrix3 start_orient;
	TMatrix4 view;
	TMatrix3 cam_orient;
	TVec3 pos;

	bool active;
	float speed;

	TVec2i middle_pos;
	float mouse_sensitivity;

	TFPSCameraPrivate()
	{

	}

	TFPSCameraPrivate(TVec3 use_pos, TVec3 use_dir, TVec3 use_up)
	{
		active = false;
		speed = 20;
		mouse_sensitivity = 0.006f;
		yaw = 0;
		pitch = 0;//TODO dir,up
		//start_orient.SetIdentity();
		pos = use_pos;
		UpdateView();
	}

	void UpdateView()
	{
		TVec3 dir_z, dir_x, dir;
		dir_z = TVec3(sin(yaw), 0, cos(yaw));
		dir_x = TVec3(dir_z[2], 0, -dir_z[0]);
		dir = dir_z.GetRotated(dir_x, pitch);

		TMatrix3 m(dir_x, dir.Cross(dir_x), dir);
		cam_orient = m;

		m.Transpose();
		view = TMatrix4(m, -(m*pos));
	}

	TMatrix4 GetView()
	{
		return view;
	}

	void KeyDown(TFPSCamera::Key key, float time, float mult)
	{
		float s = speed * mult;
		switch (key)
		{
		case TFPSCamera::Key::Left:
			pos -= cam_orient[0] * s*time;
			break;
		case TFPSCamera::Key::Right:
			pos += cam_orient[0] * s*time;
			break;
		case TFPSCamera::Key::Up:
			pos -= cam_orient[2] * s*time;
			break;
		case TFPSCamera::Key::Down:
			pos += cam_orient[2] * s*time;
			break;
                default:
                    break;
		}
	}

	void MouseMove(int x, int y)
	{
		TVec2i mouse_pos;
		TQuaternion temp;

		mouse_pos = TVec2i(x, y);
		if (mouse_pos == middle_pos)return;

		TVec2 mouse_direction;
		mouse_direction = TVec2((float)(middle_pos[0] - mouse_pos[0]), (float)(middle_pos[1] - mouse_pos[1]))*mouse_sensitivity;

		pitch += mouse_direction[1];
		yaw += mouse_direction[0];
		UpdateView();
	}
};

TFPSCamera::~TFPSCamera()
{
	delete p;
}

TFPSCamera::TFPSCamera()
{
	p = new TFPSCameraPrivate();
}

TFPSCamera::TFPSCamera(TVec3 use_pos, TVec3 use_dir, TVec3 use_up)
{
	p = new TFPSCameraPrivate(use_pos, use_dir, use_up);
}

TFPSCamera::TFPSCamera(const TFPSCamera &source)
{
	p = new TFPSCameraPrivate(*source.p);
}

void TFPSCamera::operator=(const TFPSCamera &source)
{
	*p = *source.p;
}

void TFPSCamera::UpdateView()
{
	p->UpdateView();
}

void TFPSCamera::MouseMove(int x, int y)
{
	p->MouseMove(x, y);
}

void TFPSCamera::KeyDown(Key key, float time, float mult)
{
	p->KeyDown(key, time, mult);
}

float TFPSCamera::GetYaw()
{
	return p->yaw;
}

float TFPSCamera::GetPitch()
{
	return p->pitch;
}

TMatrix4 TFPSCamera::GetView()
{
	return p->GetView();
}

TMatrix3 TFPSCamera::GetOrient()
{
	return p->cam_orient;
}

bool TFPSCamera::IsActive()
{
	return p->active;
}

void TFPSCamera::Activate(int x, int y)
{
	p->active = true;
	p->middle_pos[0] = x;
	p->middle_pos[1] = y;
}

void TFPSCamera::Deactivate()
{
	p->active = false;
}

TVec3 TFPSCamera::GetPos()const
{
	return p->pos;
}

void TFPSCamera::SetPos(const TVec3& use_pos)
{
	p->pos = use_pos;
}

TVec3 TFPSCamera::GetDir()const
{
	return -p->cam_orient[2];
}

void TFPSCamera::SetDir(const TVec3& use_dir)
{
	//TODO yaw pitch
	//dir=use_dir.GetNormalized();
};
