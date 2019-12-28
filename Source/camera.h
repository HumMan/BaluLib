#pragma once

#include "Math/vec.h"
#include "Math/matrix.h"

#include "../Include/export.h"

namespace BaluLib
{
	class TFPSCameraPrivate;
	class BALULIB_DLL_INTERFACE TFPSCamera
	{
	private:
		TFPSCameraPrivate* p;		

	public:

		TFPSCamera();
		TFPSCamera(TVec3 use_pos, TVec3 use_dir, TVec3 use_up);
		TFPSCamera(const TFPSCamera&);
		void operator=(const TFPSCamera&);
		~TFPSCamera();
		
		enum class Key
		{
			Left, Up, Right, Down, None
		};

		void UpdateView();

		void MouseMove(int x, int y);
		void KeyDown(Key key, float time, float mult);

		float GetYaw();
		float GetPitch();
		
		TMatrix4 GetView();
		TMatrix3 GetOrient();
		bool IsActive();
		void Activate(int x, int y);
		TVec3 GetPos()const;
		void SetPos(const TVec3& use_pos);
		TVec3 GetDir()const;
		void SetDir(const TVec3& use_dir);
		void Deactivate();
	};

	template<class T>
	class TOrbitCamera
	{

	private:
		TVec<T, 3> up, dir, target;
		T offset_x, offset_y;
		T dist;

		TVec2i curr_pos;
		TMatrix<T, 4> view;

		bool panning;
		bool rotating;
		bool zooming;

		void UpdateView()
		{
			TVec<T, 3> pos = dir * (dist);
			TVec<T, 3> right = up.Cross(dir);
			TVec<T, 3> off_global = right * offset_x + up * offset_y;
			view.SetIdentity();
			TMatrix<T, 3> temp(right, up, dir);
			temp.Transpose();
			view.SetRotation(temp);
			view.SetOffset(-(temp*(pos + target + off_global)));
		}
	public:

		TOrbitCamera(TVec<T, 3> use_pos, TVec<T, 3> use_target, TVec<T, 3> use_up)
		{
			panning = false;
			rotating = false;
			zooming = false;
			SetView(use_pos, use_target, use_up);
		}

		void SetView(TVec<T, 3> use_pos, TVec<T, 3> use_target, TVec<T, 3> use_up)
		{
			target = use_target;
			dir = (use_pos - use_target).GetNormalized();
			dist = use_pos.Distance(use_target);
			TVec<T, 3> right = use_up.Cross(dir);
			up = dir.Cross(right);
			offset_x = 0;
			offset_y = 0;
			UpdateView();
		}
		
		TMatrix<T, 4> GetView()
		{
			return view;
		}
		TVec<T, 3> GetPos()
		{
			TVec<T, 3> right = up.Cross(dir);
			return target + dir*dist + right * offset_x + up * offset_y;
		}
		TVec<T, 3> GetDir()
		{
			return dir;
		}
		TVec<T, 3> GetTarget()
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
			TVec<T, 3> right = TVec<T, 3>(0, 0, 0);
			if (panning || rotating)
				right = up.Cross(dir);
			if (rotating)
			{
				T x = (new_pos[0] - curr_pos[0]) / 100.0;
				T y = (curr_pos[1] - new_pos[1]) / 100.0;

				if (x != 0)
				{
					dir = dir.GetRotated(up, -x);
					dir.Normalize();
				}
				if (y != 0)
				{
					dir = dir.GetRotated(right, y);
					dir.Normalize();
					up = dir.Cross(right);
					up.Normalize();
				}
			}
			if (panning)
			{
				offset_x += dist * (curr_pos[0] - new_pos[0]) / 500.0;
				offset_y += dist * (new_pos[1] - curr_pos[1]) / 500.0;
			}
			if (zooming)
			{
				dist += dist * (new_pos[1] - curr_pos[1])* 0.01;
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
}