#pragma once

#include <set>
#include <vector>
#include <functional>

#include "MPrerequisites.h"
#include "MType.h"
#include "MVector.h"
#include "MCollision.h"



#define INFINITY_DISTANCE (999999999)

namespace MAstar
{
    //----------------------------------------------------------------------
    // 타일 정보
    //----------------------------------------------------------------------
    struct MTile
    {
    public:
        // 인덱스 정보
        MIntPoint Index2D;

        // 막혀있는지
        MBOOL IsBlocked = MFALSE;
    };

    //----------------------------------------------------------------------
    // 타일 관리
    //----------------------------------------------------------------------
    class MGrid
    {
    public:
        //--------------------------------------------------------
        // 2차원 인덱스로 타일을 얻는다
        //--------------------------------------------------------
        const MTile *GetTile(const MIntPoint& inIndex2D) const {
            return GetTile(inIndex2D.X, inIndex2D.Y);
        }

        const MTile *GetTile(MINT32 inX, MINT32 inY) const;
       
    public:
        // 타일 카운트
        MIntSize TileCount;

        // 타일 데이터
        std::vector<MTile> TileList;
    };

    //----------------------------------------------------------------------
    // 경로 검색에 사용하는 노드
    //----------------------------------------------------------------------
    class MNode
    {
    public:
        // 초기화
        void InitNode(const MIntPoint& inIndex2D, const MINT32 inDistance_H);
        
        // G를 설정
        void SetDistance_G(MINT32 inDistance);

        MINT32 GetDistance_F() const {
            return Distance_F;
        }

        MINT32 GetDistance_G() const {
            return Distance_G;
        }

        const MIntPoint& GetIndex2D() const {
            return Index2D;
        }

        void SetIsClose(MBOOL inFlag) {
            IsClose = inFlag;
        }

        MBOOL GetIsClose() const {
            return IsClose;
        }

        void SetPrevNode(MNode* inNode) {
            PrevNode = inNode;
        }

        MNode* GetPrevNode() const {
            return PrevNode;
        }

    public:
        // 인덱스 정보
        MIntPoint Index2D;

        // 이전 경로 데이터
        MNode* PrevNode = nullptr;

        // 거리 정보
        MINT32 Distance_H = 0;  // 노드에서 종료 위치까지 거리
        MINT32 Distance_G = 0;  // 시작 위치에서 해당 노드까지의 거리
        MINT32 Distance_F = 0;  // G + H

        MBOOL IsClose = false;
    };


    //----------------------------------------------------------------------
    // 경로 검색 처리
    //----------------------------------------------------------------------
    class MPathFinder
    {
    public:
        MPathFinder();
        ~MPathFinder();

    public:
        // 경로 찾기
        void FindPath(std::vector<MIntPoint>& inList, const MGrid* inGrid, const MIntPoint& inStartIndex2D, const MIntPoint & inEndIndex2D);

        // 2D경로 찾기 
        void FindPath(std::vector<MVector2>& inList, const MVector2& inGridPos, float inTileSize, const MGrid* inGrid, const MVector2& inStartPos, const MVector2& inEndPos, MFLOAT inRadius);

    protected:
        // 대상 위치의 노드를 얻는다
        MNode* GetNode(const MIntPoint& inIndex2D);

        // 다음 체크 노드를 얻는다
        MNode* GetNextCheckNode();

        // 주변 노드 갱신
        void UpdateAroundNode(const MGrid* inGrid, MNode* inBaseNode);

        // 방향으로 거리값을 얻는다
        MINT32 GetGridDistanceByDirection(MINT32 inX, MINT32 inY);

        // 위치 정보로 인덱스정보를 얻는다
        MIntPoint GetIndex2DByPosition(const MVector2& inGridPos, MFLOAT inTileSize, const MVector2& inPos);
        MVector2 GetLeftTopPosByIndex2D(const MVector2& inGridPos, MFLOAT inTileSize, const MIntPoint& inIndex2D);
        MVector2 GetCenterPosByIndex2D(const MVector2& inGridPos, MFLOAT inTileSize, const MIntPoint& inIndex2D);

        // 인자로 들어오는 정보로 사각형 라인을 만들어서 막히는 부분이 있는지 체크
        MBOOL CheckBlockLine(const MVector2& inGridPos, float inTileSize, const MGrid* inGrid, const MVector2& inStart, const MVector2& inEnd, MFLOAT inRadius);
        
    protected:
        // 경로 찾기시 사용할 노드 풀
        std::stack<MNode*> NodePool;

        // 사용하는 노드맵
        std::map<MIntPoint, MNode*> UseNodeMap;

        // 열린 노드 리스트
        std::set<MNode*> OpenNodeSet;

        // 길찾기에 사용되는 임시 정보
        MIntPoint StartIndex2D;
        MIntPoint EndIndex2D;
    
    public:
        //--------------------------------------------------------------
        // 각 콜백
        //--------------------------------------------------------------
        // 블럭 라인 출력
        std::function<void(const MCollision::MBox2D&)> OnDrawBlockLine;

        // 체크 블럭 출력
        std::function<void(const MCollision::MBox2D&)> OnDrawCheckBlock;
    };
};