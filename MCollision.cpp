#include "MCollision.h"


namespace MCollision
{
	//---------------------------------------------------------
	// Box2D
	//---------------------------------------------------------
	MBox2D::MBox2D(const MVector2& inLT, const MVector2& inRT, const MVector2& inLB, const MVector2& inRB)
	{
		LT = inLT;
		RT = inRT;
		LB = inLB;
		RB = inRB;
	}

	MBox2D::MBox2D(const MVector2& inStart, const MVector2& inEnd, MFLOAT inHalfWidth)
	{
		MakeBox(inStart, inEnd, inHalfWidth);
	}

	MVector2 MBox2D::GetCenterPos() const
	{
		MVector2 t1 = RT - LT;
		MVector2 t2 = LB - LT;

		return LT + ((t1 + t2) * 0.5f);
	}

	MVector2 MBox2D::GetRightVector() const
	{
		return (RT - LT) * 0.5f;
	}

	MVector2 MBox2D::GetUpVector() const
	{
		return (LT - LB) * 0.5f;
	}

	void MBox2D::MakeBox(const MVector2& inStart, const MVector2& inEnd, MFLOAT inHalfWidth)
	{
		MVector2 leftVec;
		{
			MVector3 start3D(inStart.X, inStart.Y, 0);
			MVector3 end3D(inEnd.X, inEnd.Y, 0);

			// 방향 벡터
			MVector3 dir3D = end3D - start3D;
			dir3D.Normalize();

			// 왼쪽 벡터
			MVector3 leftVec3D = MVector3::CrossProduct(dir3D, MVector3(0, 0, 1));
			leftVec.Set(leftVec3D.X, leftVec3D.Y);
			leftVec.Normalize();
			leftVec *= inHalfWidth;
		}

		// 각 위치 설정
		LT = inEnd + leftVec;
		RT = inEnd - leftVec;
		LB = inStart + leftVec;
		RB = inStart - leftVec;
	}


	//---------------------------------------------------------
	// Logic
	//---------------------------------------------------------
	MBOOL CheckOBB(const MBox2D& inBox1, const MBox2D& inBox2)
	{
		// 사용할 벡터 정보
		std::array<MVector2, 4> valueList;		// 벡터 정보
		std::array<MVector2, 4> axisList;		// 축 정보(정규화)

		// 추가 함수
		auto AddFunc = [&axisList, &valueList](MINT32 inIndex, const MVector2& inVector)
		{
			valueList[inIndex] = inVector;
			axisList[inIndex] = inVector.GetNormal();
		};

		// 박스1의 위 / 오른쪽 벡터 추가
		AddFunc(0, inBox1.GetUpVector());
		AddFunc(1, inBox1.GetRightVector());

		// 박스2의 위 / 오른쪽 벡터 추가
		AddFunc(2, inBox2.GetUpVector());
		AddFunc(3, inBox2.GetRightVector());
		

		// 중앙 위치의 거리 벡터
		MVector2 distVector = inBox2.GetCenterPos() - inBox1.GetCenterPos();

		
		// 각 축 벡터에 정보를 투영하여 
		for (MINT32 i = 0; i < 4; ++i)
		{
			MFLOAT sum = 0;

			// 사용할 축을 얻는다
			const MVector2& axis = axisList[i];

			// 벡터를 축에 투영
			for (MINT32 c = 0; c < 4; ++c)
			{
				MFLOAT temp = MVector2::DotProduct(axis, valueList[c]);
				sum += abs(temp);
			}

			// 거리를 투영
			MFLOAT distance = MVector2::DotProduct(axis, distVector);
			distance = abs(distance);

			// 한 축이라도 
			// 합이 거리보다 작다면 충돌
			if (sum < distance) {
				return MFALSE;
			}
		}
		
		return MTRUE;
	}
};
