#include "AStarDungeon.h"
#include "AStar.h"

// execute: astardungeon.exe map.txt
int main(int argc, const char* argv[])
{
	if (argc < 2)
	{
		std::cout << "Pass the path to a valid map as the first argument." << std::endl;
		return 1;
	}

	AStarDungeon pathFinder;
	if (!pathFinder.LoadMap(argv[1]))
	{
		std::cout << "The provided map was invalid." << std::endl;
	}
	pathFinder.MarkPathToGoal();
	pathFinder.PrintMap();

	std::cout << "Length: " << pathFinder.GetPathLength();

	std::cin.get();
	return 0;
}