#pragma once

#include "MPrerequisites.h"
#include "MVector.h"


namespace MCollision
{
	//---------------------------------------------------------------
	// 박스 클래스
	//---------------------------------------------------------------
	class MBox2D
	{
	public:
		MBox2D(){}
		MBox2D(const MVector2& inLT, const MVector2& inRT, const MVector2& inLB, const MVector2& inRB);
		MBox2D(const MVector2& inStart, const MVector2& inEnd, MFLOAT inHalfWidth);

	public:
		// 중앙 위치를 얻는다
		MVector2 GetCenterPos() const;

		// 중앙에서 오른쪽 벡터를 얻는다
		MVector2 GetRightVector() const;

		// 중앙에서 위쪽 벡터를 얻는다
		MVector2 GetUpVector() const;

		// 시작에서 종료 위치를 중심으로 특정 넓이의 박스를 만든다
		void MakeBox(const MVector2& inStart, const MVector2& inEnd, MFLOAT inHalfWidth);

	public:
		MVector2 LT;
		MVector2 RT;
		MVector2 LB;
		MVector2 RB;
	};


	//---------------------------------------------------------------
	// 로직
	//---------------------------------------------------------------
	// OBB 체크
	MBOOL CheckOBB(const MBox2D& inBox1, const MBox2D& inBox2);
};
