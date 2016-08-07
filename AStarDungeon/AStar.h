#pragma once
#include <queue>
#include <deque>
#include <unordered_set>
#include <unordered_map>

class AStar
{
public:
	class Node
	{
	public:
		int x, y;
		float f; // estimated cost
		float g; // actual cost to get here

		Node();
		Node(int x, int y);

		bool operator==(const Node &node) const;
		bool operator!=(const Node &node) const;

		// hashing functor
		struct Hash
		{
			// use coordinates for equality, a simple noncommutative hash function
			std::size_t operator()(const Node &node) const
			{
				return 3 * node.x + node.y;
			}
		};

		// comparison functor
		struct GreaterByCost
		{
			bool operator()(const Node &lhs, const Node &rhs) const
			{
				return lhs.f > rhs.f;
			}
		};
	};

protected:
	std::vector<Node> FindPath(Node start, Node goal);
	virtual std::vector<Node> GetSuccessors(const Node &node) const = 0;
	virtual float HeuristicFunction(const Node &node1, const Node &node) const = 0;

private:
	std::deque<Node> openList; // nodes to visit
	std::unordered_set<Node, Node::Hash> closedList; // visited nodes
	std::unordered_map<Node, Node, Node::Hash> cameFrom; // backwards connections

	std::vector<Node> ReconstructPath(Node currentNode);
};