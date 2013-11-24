#include "pathfinder.h"

float DistanceBetween(int X1,int Y1, int X2, int Y2)
{
  return sqrt((float)((X2-X1)*(X2-X1))+((Y2-Y1)*(Y2-Y1)));
}

void AddToNodeList(AStarNode_List **List,AStar_Node *NodeToAdd,int *LengthPtr)
{
  AStarNode_List *newNode = malloc(sizeof(*newNode));
  newNode->node = NodeToAdd;
  newNode->next = *List;
  
  *List = newNode;
  
  if (LengthPtr)
  {
    (*LengthPtr)++;
  }
  
  return;
}

AStar_Node *CreateNode(int X,int Y,int Value,AStarNode_List **AllNodesSet)
{
  AStar_Node *ThisNode = malloc(sizeof(*ThisNode));
  ThisNode->X          = X;
  ThisNode->Y          = Y;
  ThisNode->Value      = Value;
  ThisNode->Neighbors[0]  = NULL;
  ThisNode->Neighbors[1]  = NULL;
  ThisNode->Neighbors[2]  = NULL;
  ThisNode->Neighbors[3]  = NULL;
  
  AddToNodeList(AllNodesSet,ThisNode,NULL);
  
  return ThisNode;
}

AStarNode_List *FindInNodeList(AStarNode_List *List,AStar_Node *NodeToFind)
{
  AStarNode_List *FoundNode = NULL;
  AStarNode_List *CurrentNode   = List;
  while (CurrentNode)
  {
    if ((CurrentNode->node->X == NodeToFind->X) && (CurrentNode->node->Y == NodeToFind->Y))
    {
      // Found it.
      FoundNode = CurrentNode;
      break;
    }
    
    CurrentNode = CurrentNode->next;
  }
  
  return FoundNode;
}

void RemoveFromNodeList(AStarNode_List **List,AStar_Node *NodeToRemove,int *LengthPtr)
{
  AStarNode_List *CurrentNode   = *List;
  AStarNode_List *PreviousNode  = NULL;
  while (CurrentNode)
  {
    if ((CurrentNode->node->X == NodeToRemove->X) && (CurrentNode->node->Y == NodeToRemove->Y))
    {
      // Found it.
      if (PreviousNode)
      {
        PreviousNode->next = CurrentNode->next;
      }
      else
      {
        *List = CurrentNode->next;
      }
      
      if (LengthPtr)
      {
        (*LengthPtr)--;
      }
      break;
    }
    else
    {
      PreviousNode = CurrentNode;
      CurrentNode  = CurrentNode->next;
    }
  }
  
  return;
}

void RemoveAllFromNodeList(AStarNode_List **List,int FreeNodes)
{
  if (!List)
  {
    return;
  }
  
  AStarNode_List *CurrentNode   = *List;
  AStarNode_List *NextNode;
  
  while (CurrentNode)
  {
    if (FreeNodes && CurrentNode->node)
    {
      free(CurrentNode->node);
    }
    NextNode = CurrentNode->next;
    free(CurrentNode);
    CurrentNode = NextNode;
  }
  *List = NULL;
  return;
}

AStar_Node *AStar_Find(int mapWidth,int mapHeight,int StartX,int StartY,int EndX,int EndY,int (*GetMap)(int,int),NodeDataMap *dataMap)
{
  AStar_Node     *Neighbor       = NULL;
  AStarNode_List *OpenSet        = NULL;
  AStarNode_List *ClosedSet      = NULL;
  AStarNode_List *NextInOpenSet  = NULL;
  AStar_Node   *AStar_SolvedPath = NULL;
  int           OpenSetLength    = 0;
  float         TentativeGScore  = 0;
  float         TentativeFScore  = 0;
  AStar_Node   *Current;
  float        LowestFScore;
  float        NextFScore;
  int neighbor_pos;
  AStar_Node     TempNodeToFind;
  AStarNode_List *TempNeighborNode;
  int neighborPosInLists;
  
  AllNodesGSet = NULL;
  
  
  if ((GetMap(StartX,StartY) >= 9) || (GetMap(EndX,EndY) >= 9))
  {
#if __DEBUG_PRINT
    printf("Impossible.  Either the start or end point is in a wall\n");
#endif
    return NULL;
  }
  
  Current = CreateNode(StartX,StartY,GetMap(StartX,StartY),&AllNodesGSet);
  
  dataMap[Current->X + (Current->Y*mapWidth)].GScore   = 0.0;
  dataMap[Current->X + (Current->Y*mapWidth)].FScore   = dataMap[Current->X + (Current->Y*mapWidth)].GScore + CostOfGoal(Current->X,Current->Y,EndX,EndY,GetMap);
  dataMap[Current->X + (Current->Y*mapWidth)].CameFrom = NULL;
  
  AddToNodeList(&OpenSet,Current,&OpenSetLength);
  
  while (OpenSetLength)
  {
    Current = NULL;
    NextInOpenSet = OpenSet;
    LowestFScore = dataMap[NextInOpenSet->node->X + (NextInOpenSet->node->Y*mapWidth)].FScore;
#if __DEBUG_PRINT
    printf("######## START:  Finding lowest one.  Starting with: %d, %d\n",NextInOpenSet->node->X,NextInOpenSet->node->Y);
#endif
    while (NextInOpenSet)
    {
      NextFScore = dataMap[NextInOpenSet->node->X + (NextInOpenSet->node->Y*mapWidth)].FScore;
      if (!Current || (LowestFScore > NextFScore))
      {
        Current = NextInOpenSet->node;
        LowestFScore = NextFScore;
      }
      NextInOpenSet = NextInOpenSet->next;
    }
    
#if __DEBUG_PRINT
    printf("Current: %d, %d\n",Current->X,Current->Y);
#endif
    
    if ((Current->X == EndX) && (Current->Y == EndY))
    {
      // We reached the goal.
#if __DEBUG_PRINT
      printf("Goal achieved!\n");
#endif
      AStar_SolvedPath = Current;
      break;
    }
    
#if __DEBUG_PRINT
    printf("Removing current from open set\n");
#endif
    RemoveFromNodeList(&OpenSet,Current,&OpenSetLength);
#if __DEBUG_PRINT
    printf("Adding current to closed set\n");
#endif
    if (!FindInNodeList(ClosedSet,Current))
    {
      AddToNodeList(&ClosedSet,Current,NULL);
    }
#if __DEBUG_PRINT
    printf("Analyzing neighbors\n");
#endif
    for (neighbor_pos = 0;neighbor_pos < 4;neighbor_pos++)
    {
      Neighbor = Current->Neighbors[neighbor_pos];
      if (!Neighbor)
      {
        TempNodeToFind.X = Current->X;
        TempNodeToFind.Y = Current->Y;
        switch(neighbor_pos)
        {
          case 0:
            TempNodeToFind.Y--;
            break;
          case 1:
            TempNodeToFind.X++;
            break;
          case 2:
            TempNodeToFind.Y++;
            break;
          default:
            // Assumed 3
            TempNodeToFind.X--;
        }
        
        if ((TempNodeToFind.X >= 0) && (TempNodeToFind.X < mapWidth)
            &&
            (TempNodeToFind.Y >= 0) && (TempNodeToFind.Y < mapHeight)
            && (GetMap(TempNodeToFind.X,TempNodeToFind.Y) == 1))
        {
          TempNeighborNode = FindInNodeList(AllNodesGSet,&TempNodeToFind);
          if (TempNeighborNode)
          {
#if __DEBUG_PRINT
            printf("Selecting already existing neighbor\n");
#endif
            Neighbor = TempNeighborNode->node;
          }
          else
          {
#if __DEBUG_PRINT
            printf("Creating new neighbor\n");
#endif
            Neighbor = CreateNode(TempNodeToFind.X,TempNodeToFind.Y,GetMap(TempNodeToFind.X,TempNodeToFind.Y),&AllNodesGSet);
          }
          
#if __DEBUG_PRINT
          printf("Linking current node and neighbor as, well, neighbors\n");
#endif
          Current->Neighbors[neighbor_pos] = Neighbor;
          switch(neighbor_pos)
          {
            case 0:
              Neighbor->Neighbors[2] = Current;
              break;
              
            case 1:
              Neighbor->Neighbors[3] = Current;
              break;
              
            case 2:
              Neighbor->Neighbors[0] = Current;
              break;
              
            default:
              // Assumed 3
              Neighbor->Neighbors[1] = Current;
          }
        }
        else
        {
#if __DEBUG_PRINT
          printf("Node is an obstacle or not within map.  Skipping\n");
#endif
        }
      }
      
      if (Neighbor)
      {
        TentativeGScore = dataMap[Current->X + (Current->Y * mapWidth)].GScore + DistanceBetween(Current->X,Current->Y,Neighbor->X,Neighbor->Y);
#if __DEBUG_PRINT
        printf("Tentative G score: %f\n",TentativeGScore);
#endif
        TentativeFScore = TentativeGScore + CostOfGoal(Neighbor->X,Neighbor->Y,EndX,EndY,GetMap);
#if __DEBUG_PRINT
        printf("Tentative F score: %f\n",TentativeFScore);
#endif
        
        neighborPosInLists = Neighbor->X + (Neighbor->Y * mapWidth);
        if (!FindInNodeList(ClosedSet,Neighbor) || (TentativeFScore < dataMap[neighborPosInLists].FScore))
        {
          if (!FindInNodeList(OpenSet,Neighbor) || (TentativeFScore < dataMap[neighborPosInLists].FScore))
          {
            if (!dataMap[neighborPosInLists].CameFrom)
            {
              dataMap[neighborPosInLists].CameFrom = Current;
            }
            dataMap[neighborPosInLists].GScore   = TentativeGScore;
            dataMap[neighborPosInLists].FScore   = TentativeFScore;
          
            if (!FindInNodeList(OpenSet,Neighbor))
            {
#if __DEBUG_PRINT
              printf("Added neighbor to open set\n");
#endif
              AddToNodeList(&OpenSet,Neighbor,&OpenSetLength);
            }
          }
          else
          {
#if __DEBUG_PRINT
            printf("Neighbor already in open set or tentative F score more than neighbor's F score.  Not adding to open set\n");
#endif
          }
        }
        else
        {
#if __DEBUG_PRINT
          printf("Neighbor in closed set or tentative F score is more than or equal to neighbor's F score.  Not adding to open set\n");
#endif
        }
      }
    }
  }
  
  RemoveAllFromNodeList(&OpenSet,0);
  RemoveAllFromNodeList(&ClosedSet,0);
  
  return AStar_SolvedPath;
}
