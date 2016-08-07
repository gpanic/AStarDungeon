#include "AStar.h"

AStar::Node::Node() : x(-1), y(-1) {}
AStar::Node::Node(int x, int y) : x(x), y(y) {}

bool AStar::Node::operator==(const Node &node) const
{
	return x == node.x && y == node.y;
}

bool AStar::Node::operator!=(const Node &node) const
{
	return !(*this == node);
}

// assumes monotonic heuristic, uses functions push_heap and pop_heap
// to get the node with the smallest estimate
std::vector<AStar::Node> AStar::FindPath(Node start, Node goal)
{
	start.f = start.g = 0;
	openList.push_back(start);

	Node currentNode;

	while (!openList.empty())
	{
		// get the node with the smallest estimate
		std::pop_heap(openList.begin(), openList.end(), Node::GreaterByCost());
		currentNode = openList.back();
		openList.pop_back();

		if (currentNode == goal) break;

		// node was visited
		closedList.insert(currentNode);

		std::vector<Node> successors = GetSuccessors(currentNode);
		for (Node successor : successors)
		{
			// skip already visited nodes
			if (closedList.find(successor) != closedList.end()) continue;

			// remember connection to successor
			cameFrom[successor] = currentNode;

			// update sucessor estimate and cost
			successor.g = currentNode.g + 1; // always move by one tile
			successor.f = successor.g + HeuristicFunction(successor, goal);

			// add successor to the open list if wasn't added in a previous iteration
			if (std::find(openList.begin(), openList.end(), successor) == openList.end())
			{
				openList.push_back(successor);
				std::push_heap(openList.begin(), openList.end(), Node::GreaterByCost());
			}
		}
	}

	return ReconstructPath(currentNode);
}

std::vector<AStar::Node> AStar::ReconstructPath(Node currentNode)
{
	std::vector<Node> path;
	path.push_back(currentNode);

	// go through the list of backwards connections until the starting node
	while (cameFrom.find(currentNode) != cameFrom.end())
	{
		currentNode = cameFrom[currentNode];
		path.push_back(currentNode);
	}

	return path;
}