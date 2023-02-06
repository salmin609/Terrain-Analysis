#include <pch.h>
#include "Terrain/TerrainAnalysis.h"
#include "Terrain/MapMath.h"
#include "Agent/AStarAgent.h"
#include "Terrain/MapLayer.h"
#include "Projects/ProjectThree.h"

#include <iostream>

enum class NeighborKind
{
	Left = 0,
	Right,
	Down,
	Up,
	UpLeft,
	UpRight,
	DownLeft,
	DownRight
};

bool ProjectThree::implemented_fog_of_war() const // extra credit
{
	return false;
}

bool CheckValidGrid(GridPos gridPos)
{
	if(terrain->is_valid_grid_position(gridPos))
		if (!terrain->is_wall(gridPos))
			return true;
	return false;
}

float distance_to_closest_wall(int row, int col)
{
	/*
		Check the euclidean distance from the given cell to every other wall cell,
		with cells outside the map bounds treated as walls, and return the smallest
		distance.  Make use of the is_valid_grid_position and is_wall member
		functions in the global terrain to determine if a cell is within map bounds
		and a wall, respectively.
	*/
	const GridPos curr{ row, col };
	double closestSoFar = 1000.0;
	const int mapWidth = terrain->get_map_width();
	const int mapHeight = terrain->get_map_height();

	for (int i = 0; i < mapHeight; ++i)
	{
		for (int j = 0; j < mapWidth; ++j)
		{
			GridPos gridPos{ j, i };

			if (terrain->is_valid_grid_position(gridPos))
			{
				if (terrain->is_wall(gridPos))
				{
					double xDiff = curr.row - gridPos.row;
					double yDiff = curr.col - gridPos.col;

					const double distance = sqrt(xDiff * xDiff + yDiff * yDiff);

					if (distance < closestSoFar)
						closestSoFar = distance;
				}
			}
		}
	}

	return static_cast<float>(closestSoFar);
}



bool is_clear_path(int row0, int col0, int row1, int col1)
{
	/*
		Two cells (row0, col0) and (row1, col1) are visible to each other if a line
		between their centerpoints doesn't intersect the four boundary lines of every
		wall cell.  You should puff out the four boundary lines by a very tiny amount
		so that a diagonal line passing by the corner will intersect it.  Make use of the
		line_intersect helper function for the intersection test and the is_wall member
		function in the global terrain to determine if a cell is a wall or not.
	*/

	//build line
	//line 0
	const Vec2 line0p0{ static_cast<float>(row0), static_cast<float>(col0) };
	const Vec2 line0p1{ static_cast<float>(row1), static_cast<float>(col1) };

	const int mapWidth = terrain->get_map_width();
	const int mapHeight = terrain->get_map_height();

	for (int i = 0; i < mapHeight; ++i)
	{
		for (int j = 0; j < mapWidth; ++j)
		{
			GridPos pos{ j, i };

			if (terrain->is_wall(pos))
			{
				const GridPos wallPos{ j, i };

				Vec2 leftUpCorner{ static_cast<float>(wallPos.row) - 0.5f, static_cast<float>(wallPos.col) + 0.5f };
				Vec2 rightUpCorner{ static_cast<float>(wallPos.row) + 0.5f, static_cast<float>(wallPos.col) + 0.5f };
				Vec2 leftDownCorner{ static_cast<float>(wallPos.row) - 0.5f, static_cast<float>(wallPos.col) - 0.5f };
				Vec2 rightDownCorner{ static_cast<float>(wallPos.row) + 0.5f, static_cast<float>(wallPos.col) - 0.5f };

				//For each wall, build 4 boundary line.
				//left
				//leftUp, leftDown
				if (line_intersect(line0p0, line0p1, leftUpCorner, leftDownCorner))
					return false;

				//right
				//rightUp, rightDown
				if (line_intersect(line0p0, line0p1, rightUpCorner, rightDownCorner))
					return false;

				//up
				//leftUp, rightUp
				if (line_intersect(line0p0, line0p1, leftUpCorner, rightUpCorner))
					return false;

				//down
				//leftDown, rightDown
				if (line_intersect(line0p0, line0p1, leftDownCorner, rightDownCorner))
					return false;
			}
		}
	}

	// otherwise, return true
	return true;
}

void analyze_openness(MapLayer<float>& layer)
{
	/*
		Mark every cell in the given layer with the value 1 / (d * d),
		where d is the distance to the closest wall or edge.  Make use of the
		distance_to_closest_wall helper function.  Walls should not be marked.
	*/
	//Assume size of layer == size of map
	const int mapWidth = terrain->get_map_width();
	const int mapHeight = terrain->get_map_height();

	for (int i = 0; i < mapHeight; ++i)
	{
		for (int j = 0; j < mapWidth; ++j)
		{
			//Find closest distance 2 wall.
			if (!terrain->is_wall(j, i))
			{
				const float closestDistanceToWall = distance_to_closest_wall(j, i);

				//Find closest distance 2 edge.
				const float distanceToLeftEdge = static_cast<float>(j + 1);
				const float distanceToRightEdge = static_cast<float>((mapWidth)-j);
				const float distanceToDownEdge = static_cast<float>(i + 1);
				const float distanceToUpEdge = static_cast<float>((mapHeight)-i);

				float minSoFar;
				minSoFar = std::min(closestDistanceToWall, distanceToLeftEdge);
				minSoFar = std::min(minSoFar, distanceToRightEdge);
				minSoFar = std::min(minSoFar, distanceToDownEdge);
				minSoFar = std::min(minSoFar, distanceToUpEdge);


				float settingValue = 1.f / (minSoFar * minSoFar);

				//Set every cell of given layer with value 1 / d*d;
				layer.set_value(j, i, settingValue);
			}

		}
	}
}

void analyze_visibility(MapLayer<float>& layer)
{
	/*
		Mark every cell in the given layer with the number of cells that
		are visible to it, divided by 160 (a magic number that looks good).  Make sure
		to cap the value at 1.0 as well.

		Two cells are visible to each other if a line between their centerpoints doesn't
		intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
		helper function.
	*/

	const int mapWidth = terrain->get_map_width();
	const int mapHeight = terrain->get_map_height();
	std::vector<std::vector<int>> visibleCount;

	visibleCount.resize(mapHeight);

	for (int i = 0; i < mapHeight; ++i)
	{
		visibleCount[i].resize(mapWidth);

		for (int j = 0; j < mapWidth; ++j)
		{
			visibleCount[i][j] = 0;
		}
	}

	for (int i = 0; i < mapHeight; ++i)
	{
		for (int j = 0; j < mapWidth; ++j)
		{
			const GridPos currPos{ j, i };

			const bool isCurrWall = terrain->is_wall(currPos);

			if (!isCurrWall)
			{
				for (int y = 0; y < mapHeight; ++y)
				{
					for (int x = 0; x < mapWidth; ++x)
					{
						const GridPos comparePos{ x, y };

						const bool isCompareWall = terrain->is_wall(comparePos);

						if (!isCompareWall)
						{
							const bool isVisible = is_clear_path(currPos.row, currPos.col,
								comparePos.row, comparePos.col);

							if (isVisible)
								visibleCount[y][x]++;
						}
					}
				}
			}
		}
	}

	for (int i = 0; i < mapHeight; ++i)
	{
		for (int j = 0; j < mapWidth; ++j)
		{
			float value = static_cast<float>(visibleCount[i][j]) / 160.f;

			layer.set_value(j, i, value);
		}
	}
}

void analyze_visible_to_cell(MapLayer<float>& layer, int row, int col)
{
	/*
		For every cell in the given layer mark it with
		1.0 if it is visible to the given cell,
		0.5 if it isn't visible but is next to a visible cell,
		or 0.0 otherwise.

		Two cells are visible to each other if a line between their centerpoints doesn't
		intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
		helper function.
	*/

	const int mapWidth = terrain->get_map_width();
	const int mapHeight = terrain->get_map_height();

	for(int i = 0; i < mapHeight; ++i)
	{
		for(int j = 0; j < mapWidth; ++j)
		{
			layer.set_value(j, i, 0.0f);
		}
	}

	for (int i = 0; i < mapHeight; ++i)
	{
		for (int j = 0; j < mapWidth; ++j)
		{
			const GridPos gridPos{ j, i };

			if (terrain->is_valid_grid_position(gridPos))
			{
				if (!terrain->is_wall(gridPos))
				{
					//layer.set_value(gridPos.row, gridPos.col, 0.0f);
					const bool isVisible = is_clear_path(row, col, gridPos.row, gridPos.col);

					if (isVisible)
					{
						layer.set_value(j, i, 1.0f);

						const GridPos upPos{ j, i + 1 };
						const GridPos downPos{ j, i - 1 };
						const GridPos leftPos{ j - 1, i };
						const GridPos rightPos{ j + 1, i };

						const GridPos upLeft{ upPos.row - 1, upPos.col };
						const GridPos upRight{ upPos.row + 1, upPos.col };
						const GridPos downLeft{ downPos.row - 1, downPos.col };
						const GridPos downRight{ downPos.row + 1, downPos.col };

						std::vector neighbors{ upPos, downPos, leftPos, rightPos,
						upLeft, upRight, downLeft, downRight };

						std::vector neighborKinds{
							NeighborKind::Up, NeighborKind::Down,NeighborKind::Left, NeighborKind::Right,
							NeighborKind::UpLeft, NeighborKind::UpRight, NeighborKind::DownLeft, NeighborKind::DownRight
						};

						const int neighborsSize = static_cast<int>(neighbors.size());

						for (int k = 0; k < neighborsSize; ++k)
						{
							const GridPos neighbor = neighbors[k];
							NeighborKind neighborKind = neighborKinds[k];

							

							if (terrain->is_valid_grid_position(neighbor))
							{
								if (!terrain->is_wall(neighbor))
								{
									const bool isNeighborDiagonal = static_cast<int>(neighborKind) > 3;
									if (isNeighborDiagonal)
									{
										//Needs additional wall check
										switch (neighborKind)
										{
										case NeighborKind::UpLeft:
											if (terrain->is_wall(upPos) && terrain->is_wall(leftPos))
												continue;
											break;
										case NeighborKind::UpRight:
											if (terrain->is_wall(upPos) && terrain->is_wall(rightPos))
												continue;
											break;
										case NeighborKind::DownLeft:
											if (terrain->is_wall(downPos) && terrain->is_wall(leftPos))
												continue;
											break;
										case NeighborKind::DownRight:
											if (terrain->is_wall(downPos) && terrain->is_wall(rightPos))
												continue;
											break;
										default:;
											break;
										}
									}


									const float neighborValue = layer.get_value(neighbor.row, neighbor.col);

									if (neighborValue < 0.49f)
									{
										layer.set_value(neighbor.row, neighbor.col, 0.5f);
									}
								}
							}
						}
					}
				}

			}
		}
	}
}

void analyze_agent_vision(MapLayer<float>& layer, const Agent* agent)
{
	/*
		For every cell in the given layer that is visible to the given agent,
		mark it as 1.0, otherwise don't change the cell's current value.

		You must consider the direction the agent is facing.  All of the agent data is
		in three dimensions, but to simplify you should operate in two dimensions, the XZ plane.

		Take the dot product between the view vector and the vector from the agent to the cell,
		both normalized, and compare the cosines directly instead of taking the arccosine to
		avoid introducing floating-point inaccuracy (larger cosine means smaller angle).

		Give the agent a field of view slighter larger than 180 degrees.

		Two cells are visible to each other if a line between their centerpoints doesn't
		intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
		helper function.
	*/
	const int mapWidth = terrain->get_map_width();
	const int mapHeight = terrain->get_map_height();

	Vec3 viewVec = agent->get_forward_vector();
	viewVec.Normalize();
	viewVec.y = 0.f;

	Vec3 agentWorldPos = agent->get_position();

	GridPos agentGridPos = terrain->get_grid_position(agent->get_position());

	for(int i = 0; i < mapHeight; ++i)
	{
		for(int j = 0; j < mapWidth; ++j)
		{
			GridPos cellPos{ j, i };
			Vec3 cellWorldPos = terrain->get_world_position(cellPos);
			Vec3 agentToCell = cellWorldPos - agentWorldPos;
			agentToCell.Normalize();
			agentToCell.y = 0.f;
			
			float dotProduct = agentToCell.Dot(viewVec);

			if(dotProduct >= 0.f)
			{
				if(terrain->is_valid_grid_position(cellPos))
				{
					if(!terrain->is_wall(cellPos))
					{
						if(is_clear_path(cellPos.row, cellPos.col, agentGridPos.row, agentGridPos.col))
						{
							layer.set_value(cellPos.row, cellPos.col, 1.f);
						}
					}
				}
			}
		}
	}
}

void propagate_solo_occupancy(MapLayer<float>& layer, float decay, float growth)
{
	/*
		For every cell in the given layer:

			1) Get the value of each neighbor and apply decay factor
			2) Keep the highest value from step 1
			3) Linearly interpolate from the cell's current value to the value from step 2
			   with the growing factor as a coefficient.  Make use of the lerp helper function.
			4) Store the value from step 3 in a temporary layer.
			   A float[40][40] will suffice, no need to dynamically allocate or make a new MapLayer.

		After every cell has been processed into the temporary layer, write the temporary layer into
		the given layer;
	*/
	const int mapWidth = terrain->get_map_width();
	const int mapHeight = terrain->get_map_height();
	

	bool needPropagation = false;

	for(int i = 0; i < mapHeight; ++i)
	{
		for(int j = 0; j < mapWidth; ++j)
		{
			const float oldInfluenceVal = layer.get_value(j, i);

			if (oldInfluenceVal > 0.f)
			{
				needPropagation = true;
				break;
			}
			if (needPropagation)
				break;
		}
	}

	if(needPropagation)
	{
		//Begin Propagation
		float tempLayer[40][40];

		//initialize tempLayer

		for(int i = 0; i < 40; ++i)
		{
			for(int j = 0; j < 40; ++j)
			{
				tempLayer[i][j] = 0.f;
			}
		}


		const float sqrtVal = static_cast<float>(sqrt(2));

		for(int i = 0; i < mapHeight; ++i)
		{
			for(int j = 0; j < mapWidth; ++j)
			{
				const GridPos currPos{ j, i };

				if(CheckValidGrid(currPos))
				{
					//need to find maximum propagation from neighboring grids
					const GridPos upPos{ j, i + 1 };
					const GridPos downPos{ j, i - 1 };
					const GridPos leftPos{ j - 1, i };
					const GridPos rightPos{ j + 1, i };

					const GridPos upLeft{ upPos.row - 1, upPos.col };
					const GridPos upRight{ upPos.row + 1, upPos.col };
					const GridPos downLeft{ downPos.row - 1, downPos.col };
					const GridPos downRight{ downPos.row + 1, downPos.col };

					std::vector neighbors{ upPos, downPos, leftPos, rightPos,
					upLeft, upRight, downLeft, downRight };
					std::vector neighborKinds{
						NeighborKind::Up, NeighborKind::Down,NeighborKind::Left, NeighborKind::Right,
					NeighborKind::UpLeft, NeighborKind::UpRight, NeighborKind::DownLeft, NeighborKind::DownRight };

					const int neighborSize = static_cast<int>(neighbors.size());
					float maximumDecayedInfluenceValue = 0.f;

					//Check for every neighbor
					for(int k = 0; k < neighborSize; ++k)
					{
						const GridPos neighborPos = neighbors[k];
						const NeighborKind neighborKind = neighborKinds[k];
						const bool isNeighborDiagonal = static_cast<int>(neighborKind) > 3;

						float neighborDistance = 1.f;

						if (isNeighborDiagonal)
							neighborDistance = sqrtVal;

						//If neighbor is valid grid (not wall, valid pos)
						if(CheckValidGrid(neighborPos))
						{
							if(isNeighborDiagonal)
							{
								//Needs additional wall check
								switch(neighborKind)
								{
								case NeighborKind::UpLeft: 
									if (terrain->is_wall(upPos) || terrain->is_wall(leftPos))
										continue;
									break;
								case NeighborKind::UpRight:
									if (terrain->is_wall(upPos) || terrain->is_wall(rightPos))
										continue;
									break;
								case NeighborKind::DownLeft:
									if (terrain->is_wall(downPos) || terrain->is_wall(leftPos))
										continue;
									break;
								case NeighborKind::DownRight:
									if (terrain->is_wall(downPos) || terrain->is_wall(rightPos))
										continue;
									break;
								default: ;
									break;
								}
							}
							const float neighborValue = layer.get_value(neighborPos.row, neighborPos.col);

							//Get the influence value of each neighbor after decay.
							const float decayedInfluenceValue = neighborValue * exp(-1.f * neighborDistance * decay);

							//Keep the maximum decayed Influence value
							if (decayedInfluenceValue > maximumDecayedInfluenceValue)
								maximumDecayedInfluenceValue = decayedInfluenceValue;
						}
					}

					//Apply linear interpolation to the influence value of tile, and maximum decayed influence from all neighbors
					//with growing factor as coefficient.

					const float currGridInfluenceValue = layer.get_value(currPos.row, currPos.col);
					const float appliedInfluenceValue = lerp(currGridInfluenceValue, maximumDecayedInfluenceValue, growth);

					//Store the result to the temp layer
					tempLayer[currPos.col][currPos.row] = appliedInfluenceValue;
				}
			}
		}

		//Store temp layer to layer
		for(int i = 0; i < mapHeight; ++i)
		{
			for(int j = 0; j < mapWidth; ++j)
			{
				layer.set_value(j, i, tempLayer[i][j]);
			}
		}

	}
}

void propagate_dual_occupancy(MapLayer<float>& layer, float decay, float growth)
{
	/*
		Similar to the solo version, but the values range from -1.0 to 1.0, instead of 0.0 to 1.0

		For every cell in the given layer:

		1) Get the value of each neighbor and apply decay factor
		2) Keep the highest ABSOLUTE value from step 1
		3) Linearly interpolate from the cell's current value to the value from step 2
		   with the growing factor as a coefficient.  Make use of the lerp helper function.
		4) Store the value from step 3 in a temporary layer.
		   A float[40][40] will suffice, no need to dynamically allocate or make a new MapLayer.

		After every cell has been processed into the temporary layer, write the temporary layer into
		the given layer;
	*/

	const int mapWidth = terrain->get_map_width();
	const int mapHeight = terrain->get_map_height();


	bool needPropagation = false;

	for (int i = 0; i < mapHeight; ++i)
	{
		for (int j = 0; j < mapWidth; ++j)
		{
			const float oldInfluenceVal = layer.get_value(j, i);

			if (oldInfluenceVal > 0.f)
			{
				needPropagation = true;
				break;
			}
			if (needPropagation)
				break;
		}
	}

	if (needPropagation)
	{
		//Begin Propagation
		float tempLayer[40][40];

		//initialize tempLayer

		for (int i = 0; i < 40; ++i)
		{
			for (int j = 0; j < 40; ++j)
			{
				tempLayer[i][j] = 0.f;
			}
		}


		const float sqrtVal = static_cast<float>(sqrt(2));

		for (int i = 0; i < mapHeight; ++i)
		{
			for (int j = 0; j < mapWidth; ++j)
			{
				const GridPos currPos{ j, i };

				if (CheckValidGrid(currPos))
				{
					//need to find maximum propagation from neighboring grids
					const GridPos upPos{ j, i + 1 };
					const GridPos downPos{ j, i - 1 };
					const GridPos leftPos{ j - 1, i };
					const GridPos rightPos{ j + 1, i };

					const GridPos upLeft{ upPos.row - 1, upPos.col };
					const GridPos upRight{ upPos.row + 1, upPos.col };
					const GridPos downLeft{ downPos.row - 1, downPos.col };
					const GridPos downRight{ downPos.row + 1, downPos.col };

					std::vector neighbors{ upPos, downPos, leftPos, rightPos,
					upLeft, upRight, downLeft, downRight };
					
					std::vector neighborKinds{
						NeighborKind::Up, NeighborKind::Down,NeighborKind::Left, NeighborKind::Right,
					NeighborKind::UpLeft, NeighborKind::UpRight, NeighborKind::DownLeft, NeighborKind::DownRight };

					const int neighborSize = static_cast<int>(neighbors.size());
					float maximumDecayedInfluenceValue = 0.f;

					//Check for every neighbor
					for (int k = 0; k < neighborSize; ++k)
					{
						const GridPos neighborPos = neighbors[k];
						const NeighborKind neighborKind = neighborKinds[k];
						const bool isNeighborDiagonal = static_cast<int>(neighborKind) > 3;

						float neighborDistance = 1.f;

						if (isNeighborDiagonal)
							neighborDistance = sqrtVal;

						//If neighbor is valid grid (not wall, valid pos)
						if (CheckValidGrid(neighborPos))
						{
							if (isNeighborDiagonal)
							{
								//Needs additional wall check
								switch (neighborKind)
								{
								case NeighborKind::UpLeft:
									if (terrain->is_wall(upPos) || terrain->is_wall(leftPos))
										continue;
									break;
								case NeighborKind::UpRight:
									if (terrain->is_wall(upPos) || terrain->is_wall(rightPos))
										continue;
									break;
								case NeighborKind::DownLeft:
									if (terrain->is_wall(downPos) || terrain->is_wall(leftPos))
										continue;
									break;
								case NeighborKind::DownRight:
									if (terrain->is_wall(downPos) || terrain->is_wall(rightPos))
										continue;
									break;
								default:;
									break;
								}
							}
							const float neighborValue = layer.get_value(neighborPos.row, neighborPos.col);

							//Get the influence value of each neighbor after decay.
							const float decayedInfluenceValue = neighborValue * exp(-1.f * neighborDistance * decay);
							const float absDecauedInfluenceVlaue = abs(decayedInfluenceValue);

							//Keep the maximum decayed Influence value
							if (absDecauedInfluenceVlaue > maximumDecayedInfluenceValue)
								maximumDecayedInfluenceValue = absDecauedInfluenceVlaue;
						}
					}

					//Apply linear interpolation to the influence value of tile, and maximum decayed influence from all neighbors
					//with growing factor as coefficient.

					const float currGridInfluenceValue = layer.get_value(currPos.row, currPos.col);
					const float appliedInfluenceValue = lerp(currGridInfluenceValue, maximumDecayedInfluenceValue, growth);

					//Store the result to the temp layer
					tempLayer[currPos.col][currPos.row] = appliedInfluenceValue;
				}
			}
		}

		//Store temp layer to layer
		for (int i = 0; i < mapHeight; ++i)
		{
			for (int j = 0; j < mapWidth; ++j)
			{
				layer.set_value(j, i, tempLayer[i][j]);
			}
		}

	}
}

void normalize_solo_occupancy(MapLayer<float>& layer)
{
	/*
		Determine the maximum value in the given layer, and then divide the value
		for every cell in the layer by that amount.  This will keep the values in the
		range of [0, 1].  Negative values should be left unmodified.
	*/
	const int mapHeight = terrain->get_map_height();
	const int mapWidth = terrain->get_map_width();
	float highestValueSoFar = 0.f;


	//Find highest value.
	for(int i = 0; i < mapHeight; ++i)
	{
		for(int j = 0; j < mapWidth; ++j)
		{
			const GridPos gridPos{ j, i };

			//Skip the wall
			if(!terrain->is_wall(gridPos))
			{
				const float layerValue = layer.get_value(gridPos.row, gridPos.col);

				if(layerValue > highestValueSoFar)
				{
					highestValueSoFar = layerValue;
				}
			}
		}
	}

	for(int i = 0; i < mapHeight; ++i)
	{
		for(int j = 0; j < mapWidth; ++j)
		{
			const GridPos gridPos{ j, i };

			//Skip the wall
			if (!terrain->is_wall(gridPos))
			{
				const float layerValue = layer.get_value(gridPos.row, gridPos.col);

				//Ignore negative value, negative value is for enemy's fov;
				if(layerValue > 0.f)
				{
					const float newLayerValue = layerValue / highestValueSoFar;

					layer.set_value(gridPos.row, gridPos.col, newLayerValue);
				}
			}
		}
	}
}

void normalize_dual_occupancy(MapLayer<float>& layer)
{
	/*
		Similar to the solo version, but you need to track greatest positive value AND
		the least (furthest from 0) negative value.

		For every cell in the given layer, if the value is currently positive divide it by the
		greatest positive value, or if the value is negative divide it by -1.0 * the least negative value
		(so that it remains a negative number).  This will keep the values in the range of [-1, 1].
	*/
	const int mapHeight = terrain->get_map_height();
	const int mapWidth = terrain->get_map_width();
	float highestValueSoFar = 0.f;
	float leastValueSoFar = 0.f;


	//Find highest value.
	for (int i = 0; i < mapHeight; ++i)
	{
		for (int j = 0; j < mapWidth; ++j)
		{
			const GridPos gridPos{ j, i };

			//Skip the wall
			if (!terrain->is_wall(gridPos))
			{
				const float layerValue = layer.get_value(gridPos.row, gridPos.col);

				if (layerValue > highestValueSoFar)
				{
					highestValueSoFar = layerValue;
				}
				else if(layerValue < leastValueSoFar)
				{
					leastValueSoFar = layerValue;
				}
			}
		}
	}

	for (int i = 0; i < mapHeight; ++i)
	{
		for (int j = 0; j < mapWidth; ++j)
		{
			const GridPos gridPos{ j, i };

			//Skip the wall
			if (!terrain->is_wall(gridPos))
			{
				const float layerValue = layer.get_value(gridPos.row, gridPos.col);

				//Ignore negative value, negative value is for enemy's fov;
				if (layerValue > 0.f)
				{
					const float newLayerValue = layerValue / highestValueSoFar;

					layer.set_value(gridPos.row, gridPos.col, newLayerValue);
				}
				else
				{
					const float denominator = -1.f * leastValueSoFar;
					const float newLayerValue = layerValue / denominator;

					layer.set_value(gridPos.row, gridPos.col, newLayerValue);
				}
			}
		}
	}
	
}

void enemy_field_of_view(MapLayer<float>& layer, float fovAngle, float closeDistance, float occupancyValue, AStarAgent* enemy)
{
	/*
		First, clear out the old values in the map layer by setting any negative value to 0.
		Then, for every cell in the layer that is within the field of view cone, from the
		enemy agent, mark it with the occupancy value.  Take the dot product between the view
		vector and the vector from the agent to the cell, both normalized, and compare the
		cosines directly instead of taking the arccosine to avoid introducing floating-point
		inaccuracy (larger cosine means smaller angle).

		If the tile is close enough to the enemy (less than closeDistance),
		you only check if it's visible to enemy.  Make use of the is_clear_path
		helper function.  Otherwise, you must consider the direction the enemy is facing too.
		This creates a radius around the enemy that the player can be detected within, as well
		as a fov cone.
	*/

	const int mapHeight = terrain->get_map_height();
	const int mapWidth = terrain->get_map_width();

	//clear out the old values in the map layer by setting any negative value to 0.
	for(int i = 0; i < mapHeight; ++i)
	{
		for(int j = 0; j < mapWidth; ++j)
		{
			GridPos gridPos{ j, i };
			float value = layer.get_value(gridPos.row, gridPos.col);

			//If negative value
			if(value < 0.f)
			{
				//Set to zero
				layer.set_value(gridPos.row, gridPos.col, 0.f);
			}
		}
	}

	//Then, for every cell in the layer that is within the fov cone, 
	//enemy agent forward, (WorldGridPos) - (enemyPos)
	//dot product, if front, mark with occupancy value

	Vec3 enemyForwardVector = enemy->get_forward_vector();
	enemyForwardVector.Normalize();
	enemyForwardVector.y = 0.f;

	Vec3 enemyPosition = enemy->get_position();
	GridPos enemyPositionInGrid = terrain->get_grid_position(enemyPosition);

	for (int i = 0; i < mapHeight; ++i)
	{
		for (int j = 0; j < mapWidth; ++j)
		{
			GridPos gridPos{ j, i };
			Vec3 gridPosinWorld = terrain->get_world_position(gridPos);
			Vec3 enemyToGrid = gridPosinWorld - enemyPosition;
			enemyToGrid.Normalize();
			enemyToGrid.y = 0.f;

			float dotProduct = enemyToGrid.Dot(enemyForwardVector);


			const float distanceBtwEnemyAndGrid = Vec3::Distance(enemyPosition, gridPosinWorld);

			if(distanceBtwEnemyAndGrid < closeDistance)
			{
				bool isClearPath = is_clear_path(gridPos.row, gridPos.col, enemyPositionInGrid.row, enemyPositionInGrid.col);

				if (isClearPath)
				{
					layer.set_value(gridPos.row, gridPos.col, occupancyValue);
				}
			}
			else
			{
				if(dotProduct >= 0.f)
				{
					bool isClearPath = is_clear_path(gridPos.row, gridPos.col, enemyPositionInGrid.row, enemyPositionInGrid.col);

					if (isClearPath)
					{
						layer.set_value(gridPos.row, gridPos.col, occupancyValue);
					}
				}
			}
		}
	}


}

bool enemy_find_player(MapLayer<float>& layer, AStarAgent* enemy, Agent* player)
{
	/*
		Check if the player's current tile has a negative value, ie in the fov cone
		or within a detection radius.
	*/

	const auto& playerWorldPos = player->get_position();

	const auto playerGridPos = terrain->get_grid_position(playerWorldPos);

	// verify a valid position was returned
	if (terrain->is_valid_grid_position(playerGridPos) == true)
	{
		if (layer.get_value(playerGridPos) < 0.0f)
		{
			return true;
		}
	}

	// player isn't in the detection radius or fov cone, OR somehow off the map
	return false;
}

bool enemy_seek_player(MapLayer<float>& layer, AStarAgent* enemy)
{
	/*
		Attempt to find a cell with the highest nonzero value (normalization may
		not produce exactly 1.0 due to floating point error), and then set it as
		the new target, using enemy->path_to.

		If there are multiple cells with the same highest value, then pick the
		cell closest to the enemy.

		Return whether a target cell was found.
	*/

	//Attempt to find highest non zero value
	//set that as the new target

	const int mapHeight = terrain->get_map_height();
	const int mapWidth = terrain->get_map_width();

	float highestSoFar = 0.f;
	std::vector<GridPos> highestGrids;

	for(int i = 0; i < mapHeight; ++i)
	{
		for(int j = 0; j < mapWidth; ++j)
		{
			const GridPos curr{ j, i };
			const float value = layer.get_value(curr.row, curr.col);

			if(value >= highestSoFar)
			{
				//Needs to determine same highest value
				const float valueDiff = value - highestSoFar;

				//means same value
				if(valueDiff < std::numeric_limits<float>::epsilon())
				{
					highestGrids.push_back(curr);
				}
				else
				{
					//value is just larger
					//if so, clear vectors
					highestGrids.clear();

					highestGrids.push_back(curr);

					//Modify highestSoFar
					highestSoFar = value;
				}
			}
		}
	}

	//now highestGrids contain highest value grids.
	const int highestGridsSize = static_cast<int>(highestGrids.size());

	if(highestGridsSize > 0)
	{
		GridPos closestGrid = highestGrids[0];
		const Vec3 temp = terrain->get_world_position(closestGrid);
		const Vec3 enemyPos = enemy->get_position();

		float closestGridDistanceSoFar = Vec3::Distance(temp, enemyPos);


		for(int i = 1; i < highestGridsSize; ++i)
		{
			/*
				If there are multiple cells with the same highest value, then pick the
				cell closest to the enemy.
			*/
			GridPos currPos = highestGrids[i];
			Vec3 currPosInWorld = terrain->get_world_position(currPos);

			const float distance = Vec3::Distance(currPosInWorld, enemyPos);

			if(distance < closestGridDistanceSoFar)
			{
				closestGridDistanceSoFar = distance;
				closestGrid = currPos;
			}
		}
		enemy->path_to(terrain->get_world_position(closestGrid));


		return true; // REPLACE THIS
	}
	return false;
}
