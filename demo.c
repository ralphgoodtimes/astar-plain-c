#include <stdio.h>

#include "pathfinder.h"

AStarNode_List *AllNodesGSet;

#define MAP_WIDTH  20
#define MAP_HEIGHT 20

// Map gotten from here:
// https://github.com/justinhj/astar-algorithm-cpp/blob/master/findpath.cpp

int map[ /*MAP_WIDTH * MAP_HEIGHT*/ ] = 
{
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
	1,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,1,
	1,9,9,1,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,
	1,9,9,1,9,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,
	1,9,9,1,9,1,9,9,1,9,1,9,1,1,1,1,9,9,1,1,
	1,9,1,1,9,1,1,1,1,9,1,1,1,1,9,1,1,1,1,1,
	1,9,9,1,9,1,1,1,1,1,1,9,9,9,9,1,1,1,1,1,
	1,9,9,1,9,9,9,9,9,1,1,1,9,9,9,9,9,9,9,1,
	1,9,1,1,1,1,1,1,1,1,1,9,1,1,1,1,1,1,1,1,
	1,9,1,9,9,9,9,9,9,9,1,1,9,9,9,9,9,9,9,1,
	1,9,1,1,1,1,9,1,1,9,1,1,1,1,1,1,1,1,1,1,
	1,9,9,9,9,9,1,9,1,9,1,9,9,9,9,9,1,1,1,1,
	1,9,1,9,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,
	1,9,1,9,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,
	1,9,1,1,1,1,9,9,1,9,1,9,1,1,1,1,9,9,1,1,
	1,9,1,1,9,1,1,1,1,9,1,1,1,1,9,1,1,1,1,1,
	1,9,9,9,9,1,1,1,1,1,1,9,9,9,9,1,1,1,1,1,
	1,1,9,9,9,9,9,9,9,9,9,9,9,9,9,1,9,9,9,9,
	1,9,1,1,1,1,1,1,1,1,1,9,1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
};

int CustomGetMap( int x, int y )
{
	if( x < 0 ||
	    x >= MAP_WIDTH ||
		 y < 0 ||
		 y >= MAP_HEIGHT
	  )
	{
		return 9;	 
	}

	return map[(y*MAP_WIDTH)+x];
}

float CostOfGoal(int X1,int Y1, int X2, int Y2,int (*GetMap)(int,int))
{
  return DistanceBetween(X1,Y1,X2,Y2);
}

int main( int argc, char *argv[])
{
  int x,y,i;
  int *map_to_display;
  
  AStar_Node *Solution;
  AStar_Node *NextInSolution;
  AStar_Node *SolutionNavigator;
  int NextInSolutionPos;
  NodeDataMap *dataMap = malloc(sizeof(*dataMap) * MAP_WIDTH * MAP_HEIGHT);
  
  for (i=0;i<MAP_WIDTH * MAP_HEIGHT;i++)
  {
    dataMap[i].GScore   = 0.0;
    dataMap[i].FScore   = 0.0;
    dataMap[i].CameFrom = NULL;
  }
  
  int StartX = 0,  StartY = 5, 
      EndX   = 7 , EndY   = 10;
  
  Solution = AStar_Find(MAP_WIDTH,MAP_HEIGHT,StartX,StartY,EndX,EndY,CustomGetMap,dataMap);
  
  if (!Solution)
  {
#if __DEBUG_PRINT
    printf("No solution was found\n");
#endif
  }
  
  map_to_display = malloc(sizeof(map_to_display)*MAP_WIDTH*MAP_HEIGHT);
  
  for (y=0;y<MAP_HEIGHT;y++)
  {
    for (x=0;x<MAP_WIDTH;x++)
    {
      map_to_display[x+y*MAP_WIDTH] = (map[x+y*MAP_WIDTH]==9)?'#':' ';
    }
  }
  
  SolutionNavigator = NULL;
  NextInSolution = Solution;
  
  // NextInSolution will actually refer to the next node from end to start (that is, we're going reverse from the target).
  if (NextInSolution)
  {
#if __DEBUG_PRINT
      printf("Backtracking from the target\n");
#endif
    do
    {
      NextInSolutionPos = NextInSolution->X + (NextInSolution->Y * MAP_WIDTH);
#if __DEBUG_PRINT
      printf("(%d, %d) --> (%d, %d)\n",NextInSolution->X,NextInSolution->Y,dataMap[NextInSolutionPos].CameFrom->X,dataMap[NextInSolutionPos].CameFrom->Y);
#endif
      NextInSolution->NextInSolvedPath = SolutionNavigator;
      SolutionNavigator = NextInSolution;
      NextInSolution = dataMap[NextInSolutionPos].CameFrom;
    }
    while ((SolutionNavigator->X != StartX) || (SolutionNavigator->Y != StartY));
  }
  
  // Let's draw the map from the starting point.
  // We could have done it in the previous section, but here it makes more sense.
  // Or you can simply invert both Start and End points, maybe.
  while (SolutionNavigator)
  {
    map_to_display[SolutionNavigator->X+SolutionNavigator->Y*MAP_WIDTH] = 'O';
    SolutionNavigator = SolutionNavigator->NextInSolvedPath;
  }
  
  map_to_display[StartX+StartY*MAP_WIDTH] = '1';
  map_to_display[EndX+EndY*MAP_WIDTH] = '2';
  
  for (y=0;y<MAP_HEIGHT;y++)
  {
    for (x=0;x<MAP_WIDTH;x++)
    {
      switch (map_to_display[x+y*MAP_WIDTH])
      {
        case '#':
          printf("#");
          break;
        case ' ':
          printf(" ");
          break;
        case '1':
          printf("1");
          break;
        case '2':
          printf("2");
          break;
        default:
          // Assumed path.
          printf("O");
      }
    }
    
    printf("\n");
  }
  
  // 
  RemoveAllFromNodeList(&AllNodesGSet,1);
  
  free(dataMap);
  free(map_to_display);
  
	return 0;
}
