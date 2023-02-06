/*
 * Author		: Ryan Kim(Sangmin Kim)
 * Date			: 2022-10-17
 * Description	: CS380 project2 pathfinding.cpp file
 */

#include <pch.h>
#include "Projects/ProjectTwo.h"
#include "P2_Pathfinding.h"

#pragma region Extra Credit
bool ProjectTwo::implemented_floyd_warshall()
{
	return false;
}

bool ProjectTwo::implemented_goal_bounding()
{
	return false;
}

bool ProjectTwo::implemented_jps_plus()
{
	return false;
}
#pragma endregion

bool AStarPather::initialize()
{

	return true; // return false if any errors actually occur, to stop engine initialization
}

void AStarPather::shutdown()
{

}



PathResult AStarPather::compute_path(PathRequest& request)
{
	if(request.newRequest)
	{
		InitializeNodes();

		start = terrain->get_grid_position(request.start);
		goal = terrain->get_grid_position(request.goal);

		Node startNode = nodes[start.col][start.row];

		AddOnList(onList::OPEN, startNode);
	}
	
	while (!openList.empty())
	{
		Node parentNode = SortOpenListAndPop();

		if (parentNode.pos == goal)
		{
			Node startNode = nodes[start.col][start.row];
			SetPath(request.path, parentNode, startNode, request.settings.rubberBanding,
				request.settings.smoothing);
			return PathResult::COMPLETE;
		}

		//Find neighboring child nodes
		std::vector<NeighborKind> neighborKinds;
		std::vector<GridPos> neighboringGrids = GetNeighboringChildPoses(parentNode.pos, neighborKinds);

		const size_t vecSize = neighboringGrids.size();
		for (int i = 0; i < vecSize; ++i)
		{
			const GridPos childPos = neighboringGrids[i];
			const NeighborKind kind = neighborKinds[i];

			Node childNode = nodes[childPos.col][childPos.row];

			if(kind == NeighborKind::DOWN || kind == NeighborKind::UP || kind == NeighborKind::LEFT || kind == NeighborKind::RIGHT)
				childNode.given = parentNode.given + 1.f;
			else
				childNode.given = parentNode.given + static_cast<float>(sqrt2);

			childNode.cost = childNode.given +
				HeuristicCost(request.settings.heuristic, childNode.pos, goal) * request.settings.weight;
			childNode.xParent = parentNode.pos.row;
			childNode.yParent = parentNode.pos.col;

 			const onList& childNodeStatus = childNode.list;

			//Child node is not on the list
			if (childNodeStatus == onList::NONE)
			{
				AddOnList(onList::OPEN, childNode);
			}
			//Means child node is on the open / closed list
			else
			{
				//Find that child node
				Node otherChildNode;

				if (childNodeStatus == onList::OPEN)
					otherChildNode = *std::find(openList.begin(), openList.end(), childNode);
				else
					otherChildNode = *std::find(closedList.begin(), closedList.end(), childNode);

				if (childNode.cost < otherChildNode.cost)
				{
					//take the old expensive one off both lists
					if (childNodeStatus == onList::OPEN)
						DeleteOnList(onList::OPEN, childNode);
					else
						DeleteOnList(onList::CLOSED, childNode);

					//put this new cheaper one on the open list
					AddOnList(onList::OPEN, childNode);
				}


			}
		}

		AddOnList(onList::CLOSED, parentNode);

		if(request.settings.debugColoring)
			SetOpenClosedGridsColor();

		if (request.settings.singleStep)
			return PathResult::PROCESSING;
	}

	return PathResult::IMPOSSIBLE;
}

/*
 * Sort and pop cheapest node from the openList.
 */
AStarPather::Node AStarPather::SortOpenListAndPop()
{
	openList.sort([](const Node& A, const Node& B)
		{
			return A.cost < B.cost;
		});

	const Node frontNode = openList.front();
	openList.pop_front();

	return frontNode;
}

/*
 * Calculate heuristic cost depending on setting.
 */
double AStarPather::HeuristicCost(const Heuristic& setting, const GridPos& curr, const GridPos& target) const
{
	switch (setting)
	{
	case Heuristic::OCTILE:
	{
		int xDiff, yDiff;

		curr.row > target.row ? xDiff = curr.row - target.row : xDiff = target.row - curr.row;
		curr.col > target.col ? yDiff = curr.col - target.col : yDiff = target.col - curr.col;

		const double firstVal = std::min(xDiff, yDiff) * sqrt2;
		const int secondVal = std::max(xDiff, yDiff);
		const int thirdVal = std::min(xDiff, yDiff);

		return firstVal + secondVal - thirdVal;
	}
	case Heuristic::CHEBYSHEV:
	{
		int xDiff, yDiff;

		curr.row > target.row ? xDiff = curr.row - target.row : xDiff = target.row - curr.row;
		curr.col > target.col ? yDiff = curr.col - target.col : yDiff = target.col - curr.col;

		return std::max(xDiff, yDiff);
	}
	case Heuristic::MANHATTAN:
	{
		int xDiff, yDiff;

		curr.row > target.row ? xDiff = curr.row - target.row : xDiff = target.row - curr.row;
		curr.col > target.col ? yDiff = curr.col - target.col : yDiff = target.col - curr.col;

		return xDiff + yDiff;
	}
	case Heuristic::EUCLIDEAN:
	{
		int xDiff, yDiff;

		curr.row > target.row ? xDiff = curr.row - target.row : xDiff = target.row - curr.row;
		curr.col > target.col ? yDiff = curr.col - target.col : yDiff = target.col - curr.col;

		return sqrt(xDiff * xDiff + yDiff * yDiff);
	}
	case Heuristic::NUM_ENTRIES:
	default:
		return 0.f;
	}

}

/*
 * Return all valid neighbor grids positions.
 */
std::vector<GridPos> AStarPather::GetNeighboringChildPoses(const GridPos& parentPos, std::vector<NeighborKind>& neighborKinds)
{
	std::vector<GridPos> result;

	//Up
	GridPos targetPos = GridPos{parentPos.row, parentPos.col + 1};

	const bool isUpMovable = isValidGrid(targetPos);

	if (isUpMovable)
	{
		result.push_back(targetPos);
		neighborKinds.push_back(NeighborKind::UP);
	}

	//Down
	targetPos = GridPos{ parentPos.row, parentPos.col - 1 };

	const bool isDownMovable = isValidGrid(targetPos);

	if (isDownMovable)
	{
		result.push_back(targetPos);
		neighborKinds.push_back(NeighborKind::DOWN);
	}

	//Left
	targetPos = GridPos{ parentPos.row - 1, parentPos.col };

	const bool isLeftMovable = isValidGrid(targetPos);

	if (isLeftMovable)
	{
		result.push_back(targetPos);
		neighborKinds.push_back(NeighborKind::LEFT);
	}

	//Right
	targetPos = GridPos{ parentPos.row + 1, parentPos.col };

	const bool isRightMovable = isValidGrid(targetPos);

	if (isRightMovable)
	{
		result.push_back(targetPos);
		neighborKinds.push_back(NeighborKind::RIGHT);
	}

	//Diagonal
	//UpLeft
	//Should check up / left movable
	if (isLeftMovable && isUpMovable)
	{
		targetPos = GridPos{ parentPos.row - 1, parentPos.col + 1 };

		if (isValidGrid(targetPos))
		{
			result.push_back(targetPos);
			neighborKinds.push_back(NeighborKind::UPLEFT);
		}
	}

	//UpRight
	if (isRightMovable && isUpMovable)
	{
		targetPos = GridPos{ parentPos.row + 1, parentPos.col + 1 };

		if (isValidGrid(targetPos))
		{
			result.push_back(targetPos);
			neighborKinds.push_back(NeighborKind::UPRIGHT);
		}
	}

	//DownLeft
	if (isLeftMovable && isDownMovable)
	{
		targetPos = GridPos{ parentPos.row - 1, parentPos.col - 1 };

		if (isValidGrid(targetPos))
		{
			result.push_back(targetPos);
			neighborKinds.push_back(NeighborKind::DOWNLEFT);
		}
	}

	//DownRight
	if (isRightMovable && isDownMovable)
	{
		targetPos = GridPos{ parentPos.row + 1, parentPos.col - 1 };

		if (isValidGrid(targetPos))
		{
			result.push_back(targetPos);
			neighborKinds.push_back(NeighborKind::DOWNRIGHT);
		}
	}

	return result;
}

/*
 * Set color openList / closedList grids
 */
void AStarPather::SetOpenClosedGridsColor()
{
	for (const auto& element : openList)
		terrain->set_color(element.pos.row, element.pos.col, Color(0.f, 0.f, 1.f));

	for (const auto& element : closedList)
		terrain->set_color(element.pos.row, element.pos.col, Color(1.f, 1.f, 0.f));

}

/*
 * Add node on Open/Close list
 */
void AStarPather::AddOnList(const onList& type, Node node)
{
	node.list = type;
	nodes[node.pos.col][node.pos.row] = node;

	if (type == onList::OPEN)
		openList.push_back(node);
	else if (type == onList::CLOSED)
		closedList.push_back(node);
}

/*
 * Delete node from Open/Close list
 */
void AStarPather::DeleteOnList(const onList& type, Node node)
{
	if (type == onList::OPEN)
		openList.remove(node);
	else if (type == onList::CLOSED)
		closedList.remove(node);

	nodes[node.pos.col][node.pos.row].list = onList::NONE;
}

/*
 *
 */
void AStarPather::SetPath(WaypointList& list, const Node& goalNode, const Node& startNode, bool enableRubberbanding,
	bool enableSmoothing)
{
	Node tempNode = goalNode;
	std::vector<Vec3> pathLists;

	while (!(tempNode == startNode))
	{
		Vec3 check = terrain->get_world_position(tempNode.pos);

		pathLists.push_back(check);

		tempNode = nodes[tempNode.yParent][tempNode.xParent];
	}

	//Set Rubberbanding
	if (pathLists.size() > 2 && enableRubberbanding)
	{
		auto beginElement = pathLists.begin();

		while (beginElement != pathLists.end() - 2)
		{
			GridPos first = terrain->get_grid_position(*beginElement);
			GridPos second = terrain->get_grid_position(*(beginElement + 1));
			GridPos third = terrain->get_grid_position(*(beginElement + 2));

			const int minRow = std::min(std::min(first.row, second.row), std::min(second.row, third.row));
			const int minCol = std::min(std::min(first.col, second.col), std::min(second.col, third.col));
			const int maxRow = std::max(std::max(first.row, second.row), std::max(second.row, third.row));
			const int maxCol = std::max(std::max(first.col, second.col), std::max(second.col, third.col));

			bool canMerge = true;

			for (int y = minCol; y <= maxCol; ++y)
			{
				for (int x = minRow; x <= maxRow; ++x)
				{
					if (terrain->is_wall(x, y))
						canMerge = false;
				}
			}

			if (canMerge)
			{
				pathLists.erase(beginElement + 1);
			}
			else
			{
				++beginElement;
			}
		}
	}
	size_t listSize = pathLists.size();

	
	if(enableSmoothing == false)
	{
		for (size_t i = 0; i < listSize; ++i)
		{
			list.push_front(pathLists[i]);
		}
		list.push_front(terrain->get_world_position(startNode.pos));
	}
	//Smoothing
	else
	{
		if(enableRubberbanding)
		{
			//Need to fill in the pathLists
			for(size_t i = 0; i < pathLists.size() - 1;)
			{
				//Compare distance btw 2 points,
				//if > 1.5, push new points
				Vec3 v1, v2;

				v1 = pathLists[i];
				v2 = pathLists[i + 1];

				float distance = Vec3::Distance(v1, v2);

				if (distance > 5.f)
				{
					//insert
					Vec3 midPoint = (v1 + v2) / 2.f;
					pathLists.insert(pathLists.begin() + (i + 1), midPoint);
				}
				else
					i++;

			}

		}
		listSize = pathLists.size();

		if(listSize > 2)
		{
			for(size_t i = 0; i < listSize - 2; ++i)
			{
				Vec3 v1, v2, v3, v4;

				v1 = pathLists[i];

				if(i == 0)
				{
					v2 = pathLists[i];
					v3 = pathLists[i + 1];
					v4 = pathLists[i + 2];
				}
				else if(i == listSize - 3)
				{
					v2 = pathLists[i + 1];
					v3 = pathLists[i + 2];
					v4 = pathLists[i + 2];
				}
				else
				{
					v2 = pathLists[i + 1];
					v3 = pathLists[i + 2];
					v4 = pathLists[i + 3];
				}

				for(float t = 0.f; t < 1.f; t += 0.25f)
				{
					Vec3 val = Vec3::CatmullRom(v1, v2, v3, v4, t);

					list.push_front(val);
				}
			}
		}
		else if(listSize == 2)
		{
			Vec3 v1, v2, v3, v4;

			v1 = pathLists[0];
			v2 = pathLists[0];
			v3 = pathLists[1];
			v4 = pathLists[1];

			for (float t = 0.f; t <= 1.f; t += 0.25f)
			{
				Vec3 val = Vec3::CatmullRom(v1, v2, v3, v4, t);

				list.push_front(val);
			}
		}
		else if(listSize == 1)
		{
			list.push_front(terrain->get_world_position(goal));
			list.push_front(terrain->get_world_position(startNode.pos));
		}

	}


}

bool AStarPather::isValidGrid(const GridPos& pos)
{
	return terrain->is_valid_grid_position(pos) && !terrain->is_wall(pos);
}

void AStarPather::InitializeNodes()
{
	openList.clear();
	closedList.clear();

	const int mapHeight = terrain->get_map_height();
	const int mapWidth = terrain->get_map_width();

	if (nodes.size() != mapHeight)
	{
		nodes.resize(mapHeight);

		for (int i = 0; i < mapHeight; ++i)
		{
			nodes[i].resize(mapWidth);
		}
	}

	for (int i = 0; i < mapHeight; ++i)
	{
		for (int j = 0; j < mapWidth; ++j)
		{
			nodes[i][j].xParent = 0;
			nodes[i][j].yParent = 0;
			nodes[i][j].cost = 0.f;
			nodes[i][j].given = 0.f;
			nodes[i][j].list = onList::NONE;
			nodes[i][j].pos = GridPos{j, i};
		}
	}
}


