#ifndef __PATHFINDER_H
#define __PATHFINDER_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "general.h"

#define __DEBUG_PRINT 0

typedef struct
{
  int X;
  int Y;
  int Value;
  void *NextInSolvedPath;   //AStar_Node*
  void *Neighbors[4];  //AStar_Node*
} AStar_Node;

typedef struct
{
  AStar_Node *node;
  void *next; //AStarNode_List*
} AStarNode_List;

typedef struct
{
  AStar_Node   *CameFrom;
  float        GScore;
  float        FScore;
} NodeDataMap;

AStarNode_List *AllNodesGSet;

float DistanceBetween(int X1,int Y1, int X2, int Y2);

void AddToNodeList(AStarNode_List **List,AStar_Node *NodeToAdd,int *LengthPtr);

AStar_Node *CreateNode(int X,int Y,int Value,AStarNode_List **AllNodesSet);

AStarNode_List *FindInNodeList(AStarNode_List *List,AStar_Node *NodeToFind);

void RemoveFromNodeList(AStarNode_List **List,AStar_Node *NodeToRemove,int *LengthPtr);

void RemoveAllFromNodeList(AStarNode_List **List,int FreeNodes);

AStar_Node *AStar_Find(int mapWidth,int mapHeight,int StartX,int StartY,int EndX,int EndY,int (*GetMap)(int,int),NodeDataMap *dataMap);

#endif
