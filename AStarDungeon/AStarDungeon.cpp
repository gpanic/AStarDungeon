#include "AStarDungeon.h"

bool AStarDungeon::LoadMap(const std::string mapFile)
{
	std::ifstream file(mapFile);
	if (!file) return false;

	std::string line;
	bool foundStart = false;
	bool foundGoal = false;
	while (file >> line)
	{
		std::vector<char> mapRow; // each line represents a map row
		for (char c : line)
		{
			if (!IsCharValid(c)) return false;

			if (c == TileType::typeStart)
			{
				if (foundStart) // allow only one start
				{
					return false;
				}
				else 
				{
					startNode.x = mapRow.size();
					startNode.y = map.size();
					foundStart = true;
				}
			}
			else if (c == TileType::typeGoal)
			{
				if (foundGoal) // allow only one goal
				{
					return false;
				}
				else
				{
					goalNode.x = mapRow.size();
					goalNode.y = map.size();
					foundGoal = true;
				}
			}

			mapRow.push_back(c);
		}
		map.push_back(mapRow);
	}

	mapLoaded = foundStart && foundGoal;
	return mapLoaded;
}

void AStarDungeon::MarkPathToGoal()
{
	if (!mapLoaded) return;
	path = FindPath(startNode, goalNode);
	for (Node n : path)
	{
		char &tile = map.at(n.y).at(n.x);
		if (tile != TileType::typeGoal && tile != TileType::typeStart)
		{
			tile = TileType::typePath;
		}
	}
}

void AStarDungeon::PrintMap() const
{
	for (auto row = map.begin(); row != map.end(); ++row)
	{
		for (auto tile = row->begin(); tile != row->end(); ++tile)
		{
			std::cout << *tile;
		}
		std::cout << std::endl;
	}
}

int AStarDungeon::GetPathLength()
{
	if (!mapLoaded) return -1;
	return path.size() - 1; // we exclude the starting node
}

bool AStarDungeon::IsCharValid(const char &c) const
{
	return c == TileType::typeNormal ||
		c == TileType::typeWall ||
		c == TileType::typeGoal ||
		c == TileType::typeStart;
}

AStarDungeon::TileType AStarDungeon::GetTileType(const Node &node) const
{
	try
	{
		return static_cast<TileType>(map.at(node.y).at(node.x));
	}
	catch (std::out_of_range)
	{
		return TileType::typeInvalid;
	}
}

bool AStarDungeon::CanMoveTo(const Node &node) const
{
	TileType type = GetTileType(node);
	return type != TileType::typeWall && type != TileType::typeInvalid;
}

std::vector<AStarDungeon::Node> AStarDungeon::GetSuccessors(const AStarDungeon::Node &node) const
{
	Node up = node;
	++up.y;
	Node right = node;
	++right.x;
	Node down = node;
	--down.y;
	Node left = node;
	--left.x;

	std::vector<Node> successors = { up, right, down, left };

	// get rid of untraversable tiles
	auto canMoveToLambda = [this](Node &n) { return !CanMoveTo(n); };
	successors.erase(std::remove_if(successors.begin(), successors.end(), canMoveToLambda), successors.end());

	return successors;
}

// we use manhattan distance because it's monotonic, fast and simple
float AStarDungeon::HeuristicFunction(const Node &node1, const Node &node2) const
{
	return std::abs(node1.x - node2.x) + std::abs(node1.y - node2.y);
}