#pragma once

class TFPSCamera
{
private:

	float yaw, 
		pitch; 
	//TMatrix3 start_orient;
	TMatrix4 view;
	TMatrix3 cam_orient;
	TVec3 pos;

	bool active;
	float speed;      

	void SetViewByMouse();

	TVec2i middle_pos;
	float mouse_sensitivity;

	void SetViewByKeyboard(float time);
	void UpdateView();
public:	
	float GetYaw(){return yaw;}
	float GetPitch(){return pitch;}
	TFPSCamera(
		TVec3 use_pos,TVec3 use_dir,TVec3 use_up);	
	TMatrix4 GetView();
	TMatrix3 GetOrient(){return cam_orient;}
	void Update(float use_time);
	bool IsActive(){return active;}
	void Activate(int x,int y)
	{
		active=true;
		middle_pos[0]=x;
		middle_pos[1]=y;
		SetCursorPos(middle_pos);
	};
	void Deactivate()
	{
		active=false;
	};
	TVec3 GetPos()const
	{
		return pos;
	};
	void SetPos(const TVec3& use_pos)
	{
		pos=use_pos;
	};
	TVec3 GetDir()const
	{
		return -cam_orient[2];
	};
	void SetDir(const TVec3& use_dir)
	{
		//TODO ������������� � yaw pitch
		//dir=use_dir.GetNormalized();
	};
};

template<class T>
class TOrbitCamera
{

private:
	TVec<T,3> up, dir, target;
	T offset_x, offset_y;
	T dist;

	TVec2i curr_pos;
	TMatrix<T,4> view;

	bool panning;
	bool rotating;
	bool zooming;

	void UpdateView()
	{
		TVec<T,3> pos = dir * (dist);
		TVec<T,3> right = up.Cross(dir);
		TVec<T,3> off_global = right * offset_x + up * offset_y;
		view.SetIdentity();
		TMatrix<T,3> temp(right,up,dir);
		temp.Transpose();
		view.SetRotation(temp);
		view.SetOffset(-(temp*(pos+target+off_global)));
	}
public:
	void SetView(TVec<T,3> use_pos,TVec<T,3> use_target,TVec<T,3> use_up)
	{
		target = use_target;
		dir = (use_pos-use_target).GetNormalized();
		dist = use_pos.Distance(use_target);
		TVec<T,3> right = use_up.Cross(dir);
		up = dir.Cross(right);
		offset_x = 0;
		offset_y = 0;
		UpdateView();
	}
	TOrbitCamera(TVec<T,3> use_pos,TVec<T,3> use_target,TVec<T,3> use_up)
	{
		panning = false;
		rotating = false;
		zooming = false;
		SetView(use_pos,use_target,use_up);
	}
	TMatrix<T,4> GetView()
	{
		return view;
	}
	TVec<T,3> GetPos()
	{
		TVec<T,3> right = up.Cross(dir);
		return target+dir*dist+right * offset_x + up * offset_y;
	}
	TVec<T,3> GetDir()
	{
		return dir;
	}
	TVec<T,3> GetTarget()
	{
		return target;
	}
	T GetDist()
	{
		return dist;
	}
	void SetDist(T use_dist)
	{
		dist = use_dist;
		UpdateView();
	}
	void OnMouseMove(TVec2i new_pos)
	{
		TVec<T,3> right = TVec<T,3>(0, 0, 0);
		if (panning || rotating)
			right = up.Cross(dir);
		if (rotating)
		{
			T x = (new_pos[0] - curr_pos[0]) / 100.0;
			T y = (curr_pos[1] - new_pos[1]) / 100.0;

			if (x != 0)
			{
				dir=dir.GetRotated(up,-x);
				dir.Normalize();
			}
			if (y != 0)
			{
				dir=dir.GetRotated(right,y);
				dir.Normalize();
				up = dir.Cross(right);
				up.Normalize();
			}
		}
		if (panning)
		{
			offset_x += dist * (curr_pos[0]-new_pos[0]) / 500.0;
			offset_y += dist * (new_pos[1] - curr_pos[1]) / 500.0;
		}
		if(zooming)
		{
			dist += dist * (new_pos[1]-curr_pos[1])* 0.01;
		}
		curr_pos = new_pos;
		if (panning || rotating || zooming) UpdateView();
	}
	void OnMouseWheel(T scroll)
	{
		dist += dist * scroll * 0.1;
		UpdateView();
	}
	void StartRotating(TVec2i mouse_pos)
	{
		curr_pos = mouse_pos;
		rotating = true;
	}
	void StartPanning(TVec2i mouse_pos)
	{
		curr_pos = mouse_pos;
		panning = true;
	}
	void EndRotating()
	{
		rotating = false;
	}
	void EndPanning()
	{
		panning = false;
	}
	void StartZooming(TVec2i mouse_pos)
	{
		curr_pos = mouse_pos;
		zooming = true;
	}
	void EndZooming()
	{
		zooming = false;
	}
};