#pragma once
#include "AStar.h"
#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <tuple>

class AStarDungeon : AStar
{
public:
	bool LoadMap(const std::string mapFile);
	void MarkPathToGoal();
	void PrintMap() const;
	int GetPathLength();

private:
	enum TileType { typeNormal = '.', typeWall = 'W', typeGoal = 'G', typeStart = 'S', typePath = '*', typeInvalid = 'I' };

	std::vector<std::vector<char>> map;
	Node startNode;
	Node goalNode;
	bool mapLoaded = false;
	std::vector<Node> path;

	
	bool IsCharValid(const char &c) const;
	TileType GetTileType(const Node &node) const;
	bool CanMoveTo(const Node &node) const;

	std::vector<Node> GetSuccessors(const Node &node) const override;
	float HeuristicFunction(const Node &node1, const Node &node2) const override;
};