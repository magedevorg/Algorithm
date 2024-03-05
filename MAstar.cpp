#include "MAstar.h"
#include "MTrace.h"
#include "MCollision.h"


namespace MAstar
{
    //---------------------------------------------------------------------------
    // Grid
    //---------------------------------------------------------------------------
    const MTile* MGrid::GetTile(MINT32 inX, MINT32 inY) const
    {
        if (inX < 0 || TileCount.X <= inX)
        {
            return nullptr;
        }

        if (inY < 0 || TileCount.Y <= inY)
        {
            return nullptr;
        }

        // 해당 인덱스가 유효하다면 정보를 리턴
        const MINT32 index = (inY * TileCount.X) + inX;
        if (index < TileList.size())
        {
            return &TileList[index];
        }

        return nullptr;
    }

    //---------------------------------------------------------------------------
    // PathData
    //---------------------------------------------------------------------------
    void MNode::InitNode(const MIntPoint &inIndex2D, const MINT32 inDistance_H)
    {
        Index2D = inIndex2D;

        // 이전 경로 데이터 초기화
        PrevNode = nullptr;

        // 경로 정보 초기화
        Distance_H = inDistance_H;
        Distance_G = INFINITY_DISTANCE;
        Distance_F = inDistance_H;

        IsClose = false;
    }

    void MNode::SetDistance_G(MINT32 inDistance)
    {
        Distance_G = inDistance;
        Distance_F = Distance_H + Distance_G;
    }

    //---------------------------------------------------------------------------
    // AStar
    //---------------------------------------------------------------------------
    MPathFinder::MPathFinder()
    {

    }

    MPathFinder::~MPathFinder()
    {
        if (MFALSE == UseNodeMap.empty())
        {
            // 남아있다면 반납이 정상적으로 이루어지지 않은것
            MASSERT(MFALSE);
        }

        MCLEAR_PTR_STACK(NodePool);
    }

    void MPathFinder::FindPath(std::vector<MIntPoint> &inList, const MGrid* inGrid, const MIntPoint &inStartIndex2D, const MIntPoint &inEndIndex2D)
    {
        // 정보 클리어
        inList.clear();

        // 시작 / 종료 타일
        const MTile* startTile = inGrid->GetTile(inStartIndex2D);
        const MTile* endTile = inGrid->GetTile(inEndIndex2D);

        if (nullptr == startTile || nullptr == endTile)
        {
            // 에러코드
            return;
        }

        StartIndex2D = inStartIndex2D;
        EndIndex2D = inEndIndex2D;

        // 시작 노드 설정
        {
            MNode* startNode = GetNode(inStartIndex2D);

            // 시작노드의 G는 0으로 설정
            startNode->SetDistance_G(0);

            // 열린 노드에 등록
            OpenNodeSet.insert(startNode);
        }
        
        // 
        while (MTRUE)
        {
            MNode* checkNode = GetNextCheckNode();
            if (nullptr == checkNode)
            {
                // 더이상 열린 노드가 없다
                break;
            }
            
            if(checkNode->GetIndex2D() == inEndIndex2D )
            {
                // 끝에 종료 위치까지 도달
                break;
            }

            // 열린셋에서제거
            OpenNodeSet.erase(checkNode);

            // 닫힘 처리
            checkNode->SetIsClose(MTRUE);

            // 정보를 갱신
            UpdateAroundNode(inGrid, checkNode);
        }


        // 결과 위치에서 역추적한다
        {
            auto findIter = UseNodeMap.find(inEndIndex2D);
            if (UseNodeMap.end() != findIter)
            {
                MNode* currentNode = findIter->second;
                while (nullptr != currentNode)
                {
                    inList.push_back(currentNode->GetIndex2D());
                    currentNode = currentNode->GetPrevNode();
                }
            }

            // 역순으로 변경
            std::reverse(inList.begin(), inList.end());
        }

        // 반납
        {
            // 사용했던 노드는 풀에 반납
            for (auto pair : UseNodeMap) {
                NodePool.push(pair.second);
            }

            UseNodeMap.clear();

            // 데이터 셋 제거
            OpenNodeSet.clear();
        }
    }

    void MPathFinder::FindPath(std::vector<MVector2>& inList, const MVector2& inGridPos, float inTileSize, const MGrid* inGrid, const MVector2& inStartPos, const MVector2& inEndPos, MFLOAT inRadius)
    {
        inList.clear();

        // 시작 / 종료 위치 인덱스를 구한다
        MIntPoint startIndex2D = GetIndex2DByPosition(inGridPos, inTileSize, inStartPos);
        MIntPoint endIndex2D = GetIndex2DByPosition(inGridPos, inTileSize, inEndPos);

        // 동일 위치인경우 그냥 결과 위치로 이동
        if (startIndex2D == endIndex2D)
        {
            inList.push_back(inEndPos);
            return;
        }

        // 인덱스 리스트를 구한다
        std::vector<MIntPoint> index2DList;
        FindPath(index2DList, inGrid, startIndex2D, endIndex2D);

        if (MTRUE == index2DList.empty()) {
            return;
        }

        // 리스트를 돌면서 중앙 지점을 구한다
        const MINT32 count = index2DList.size();
        std::vector<MVector2> positionList;
        positionList.resize(count);

        for (MINT32 i = 0; i < count; ++i) {
            positionList[i] = GetCenterPosByIndex2D(inGridPos, inTileSize, index2DList[i]);
        }

        // 마지막 위치는 종료 위치
        if (0 < count) {
            positionList[count - 1] = inEndPos;
        }
        

        // OBB 체크

        // 마지막 인덱스
        const MINT32 lastIndex = positionList.size() - 1;

        // 시작점 추가
        inList.push_back(inStartPos);

        //
        MINT32 checkIndex = 0;
        while (lastIndex != checkIndex)
        {
            MBOOL isCollision = MTRUE;

            for(MINT32 i = lastIndex; checkIndex < i; --i)
            {
                if (MFALSE == CheckBlockLine(inGridPos, inTileSize, inGrid, positionList[checkIndex], positionList[i], inRadius))
                {
                    checkIndex = i;
                    inList.push_back(positionList[i]);
                    isCollision = MFALSE;
                    break;
                }
            }

            if (MTRUE == isCollision)
            {
                ++checkIndex;
                inList.push_back(positionList[checkIndex]);
            }
        }
    }


    MNode *MPathFinder::GetNode(const MIntPoint& inIndex2D)
    {
        // 현재 사용하는 맵에 있는지 체크
        auto findIter = UseNodeMap.find(inIndex2D);
        if (UseNodeMap.end() != findIter)
        {
            // 이미 존재한다면 그거 리턴
            return findIter->second;
        }

        // 없다면 풀에서 가져옴
        MNode *node = nullptr;
        if (false == NodePool.empty())
        {
            node = NodePool.top();
            NodePool.pop();
        }
        else
        {
            // 없다면 새로 추가
            node = new MNode();
        }

        // 사용 맵에 추가
        UseNodeMap.emplace(std::make_pair(inIndex2D, node));

        // 결과 까지의 거리를 얻는다
        // 맨허튼 거리 측정
        MINT32 distanceH = (abs(EndIndex2D.X - inIndex2D.X) * 10) + (abs(EndIndex2D.Y - inIndex2D.Y) * 10);
        
        // 초기화
        node->InitNode(inIndex2D, distanceH);

        return node;
    }


    //----------------------------------------------------------------
    // 루프를 돌면서 F가 가장 작은걸 찾는다
    //----------------------------------------------------------------
    MNode* MPathFinder::GetNextCheckNode()
    {
        MINT32 minValue = INFINITY_DISTANCE;
        MNode* nextNode = nullptr;

        for (auto targetNode : OpenNodeSet)
        {
           const MINT32 distanceF = targetNode->GetDistance_F();
            
            if (distanceF < minValue)
            {
                minValue = distanceF;
                nextNode = targetNode;
            }
        }

        return nextNode;
    }

    void MPathFinder::UpdateAroundNode(const MGrid* inGrid, MNode* inBaseNode)
    {
        // 기본 인덱스 정보
        const MIntPoint baseIndex2D = inBaseNode->GetIndex2D();
        const MINT32 baseDistance_G = inBaseNode->GetDistance_G();

        // 주변 노드 루프
        for (MINT32 x = -1; x <= 1; ++x)
        {
            for (MINT32 y = -1; y <= 1; ++y)
            {
                // 자신의 노드인경우는 넘어간다
                if (0 == x && 0 == y) {
                    continue;
                }

                // 대각선을 처리하지 않는다
                if (0 != x && 0 != y) {
                    continue;
                }


                // 대상 인덱스 정보
                const MIntPoint targetIndex2D = baseIndex2D + MIntPoint(x, y);

                // 대상 노드를 얻는다
                MNode* targetNode = GetNode(targetIndex2D);

                // 닫힌노드인경우 넘어간다
                if (MTRUE == targetNode->GetIsClose()) {
                    continue;
                }

                // 체크할 타일
                const MTile* checkTile = inGrid->GetTile(targetIndex2D);
                if (nullptr == checkTile) {
                    continue;
                }

                // 대상 타일이 막힌경우 넘어간다
                if (MTRUE == checkTile->IsBlocked) {
                    continue;
                }

                // 대상 노드는 열린 노드에 추가
                OpenNodeSet.insert(targetNode);

                //----------------------------------------------------------------
                // 거리를 체크해서 가까운경우 이동 정보를 갱신
                //----------------------------------------------------------------
                
                // 거리를 얻는다
                const MINT32 gridDistance = GetGridDistanceByDirection(x, y);

                // 대상의 현재 시작
                const MINT32 targetDistance_G = targetNode->GetDistance_G();

                // 기본노드를 거쳐서 대상 노드까지의 거리를 구한다
                const MINT32 newTargetDistance_G = baseDistance_G + gridDistance;

                // 기존에 설정되어있던 거리보다 적다면 이동 정보를 갱신해준다
                if (newTargetDistance_G < targetDistance_G)
                {
                    targetNode->SetDistance_G(newTargetDistance_G);
                    targetNode->SetPrevNode(inBaseNode);
                }
            }
        }
    }

   

    MINT32 MPathFinder::GetGridDistanceByDirection(MINT32 inX, MINT32 inY)
    {
        if (0 != inX && 0 != inY) {
            return 14;
        }
        return 10;
    }

    MIntPoint MPathFinder::GetIndex2DByPosition(const MVector2& inGridPos, MFLOAT inTileSize, const MVector2& inPos)
    {
        MVector2 relativePos = inPos - inGridPos;
        return MIntPoint(relativePos.X / inTileSize, relativePos.Y / inTileSize);
    }

    MVector2 MPathFinder::GetLeftTopPosByIndex2D(const MVector2& inGridPos, MFLOAT inTileSize, const MIntPoint& inIndex2D)
    {
		return MVector2(
			inGridPos.X + (inTileSize * inIndex2D.X),
			inGridPos.Y + (inTileSize * inIndex2D.Y));
    }

    MVector2 MPathFinder::GetCenterPosByIndex2D(const MVector2& inGridPos, MFLOAT inTileSize, const MIntPoint& inIndex2D)
    {
        return MVector2(
            inGridPos.X + (inTileSize * inIndex2D.X) + (inTileSize * 0.5f),
            inGridPos.Y + (inTileSize * inIndex2D.Y) + (inTileSize * 0.5f));
    }

    MBOOL MPathFinder::CheckBlockLine(const MVector2& inGridPos, float inTileSize, const MGrid* inGrid, const MVector2& inStart, const MVector2& inEnd, MFLOAT inRadius)
    {
        MCollision::MBox2D box1(inStart, inEnd, inRadius);

        if (nullptr != OnDrawBlockLine) {
            OnDrawBlockLine(box1);
        }
     
        MIntPoint startIndex2D = GetIndex2DByPosition(inGridPos, inTileSize, inStart);
        MIntPoint endIndex2D = GetIndex2DByPosition(inGridPos, inTileSize, inEnd);

        // 시작 ~ 종료 블록 루프
        MINT32 startX = startIndex2D.X;
        MINT32 endX = endIndex2D.X;
        if (endX < startX) {
            std::swap(startX, endX);
        }

        MINT32 startY = startIndex2D.Y;
        MINT32 endY = endIndex2D.Y;
        if (endY < startY) {
            std::swap(startY, endY);
        }

        for (MINT32 x = startX; x <= endX; ++x)
        {
            for (MINT32 y = startY; y <= endY; ++y)
            {
				MVector2 leftTop = GetLeftTopPosByIndex2D(inGridPos, inTileSize, MIntPoint(x, y));

				MCollision::MBox2D box2(
					leftTop,
					leftTop + MVector2(inTileSize, 0),
					leftTop + MVector2(0, inTileSize),
					leftTop + MVector2(inTileSize, inTileSize)
				);

                if (nullptr != OnDrawCheckBlock) {
                    OnDrawCheckBlock(box2);
                }

                if (MTRUE == MCollision::CheckOBB(box1, box2))
                {
                    // 충돌한다면 대상 그리드가 장애물인지 체크
                    if (const MAstar::MTile* tile = inGrid->GetTile(x, y))
                    {
                        if (MTRUE == tile->IsBlocked) {
                            return MTRUE;
                        }
                    }
                }
            }
        }

        return MFALSE;
    }
};



