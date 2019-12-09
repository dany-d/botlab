#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>

ObstacleDistanceGrid::ObstacleDistanceGrid(void)
    : width_(0), height_(0), metersPerCell_(0.05f), cellsPerMeter_(20.0f)
{
}

void ObstacleDistanceGrid::setDistances(const OccupancyGrid &map)
{
    resetGrid(map);
    int obs_thre = 20;

    for (int y = 0; y < map.heightInCells(); ++y)
    {
        for (int x = 0; x < map.widthInCells(); ++x)
        {
            if (map(x, y) > 0)
            {
                cells_[cellIndex(x, y)] = 0.0f;
            }
            if (map(x, y) == 0)
            {
                cells_[cellIndex(x, y)] = 0.0f;
            }
            if (map(x, y) < 0)
            {
                int dis = map.widthInCells() * map.widthInCells() + map.heightInCells() * map.heightInCells();
                for (int x_m = 1; x - x_m > 0; x_m++)
                {
                    for (int y_p = 1; y + y_p < map.heightInCells(); y_p++)
                    {
                        if (map(x - x_m, y + y_p) > obs_thre)
                        {
                            if (dis > x_m * x_m + y_p * y_p)
                            {
                                dis = x_m * x_m + y_p * y_p;
                            }
                            break;
                        }
                    }
                    for (int y_m = 1; y - y_m > 0; y_m++)
                    {
                        if (map(x - x_m, y - y_m) > obs_thre)
                        {
                            if (dis > x_m * x_m + y_m * y_m)
                            {
                                dis = x_m * x_m + y_m * y_m;
                            }
                            break;
                        }
                    }
                    break;
                }
                for (int x_p = 1; x + x_p < map.widthInCells(); x_p++)
                {
                    for (int y_p = 1; y + y_p < map.heightInCells(); y_p++)
                    {
                        if (map(x + x_p, y + y_p) > obs_thre)
                        {
                            if (dis > x_p * x_p + y_p * y_p)
                            {
                                dis = x_p * x_p + y_p * y_p;
                            }
                            break;
                        }
                    }
                    for (int y_m = 1; y - y_m > 0; y_m++)
                    {
                        if (map(x + x_p, y - y_m) > obs_thre)
                        {
                            if (dis > x_p * x_p + y_m * y_m)
                            {
                                dis = x_p * x_p + y_m * y_m;
                            }
                            break;
                        }
                    }
                    break;
                }
                for (int y_p = 1; y + y_p < map.heightInCells(); y_p++)
                {
                    if (map(x, y + y_p) > obs_thre)
                    {
                        if (dis > y_p * y_p)
                        {
                            dis = y_p * y_p;
                        }
                        break;
                    }
                }
                for (int y_m = 1; y - y_m > 0; y_m++)
                {
                    if (map(x, y - y_m) > obs_thre)
                    {
                        if (dis > y_m * y_m)
                        {
                            dis = y_m * y_m;
                        }
                        break;
                    }
                }
                for (int x_p = 1; x + x_p < map.widthInCells(); x_p++)
                {
                    if (map(x + x_p, y) > obs_thre)
                    {
                        if (dis > x_p * x_p)
                        {
                            dis = x_p * x_p;
                        }
                        break;
                    }
                }
                for (int x_m = 1; x - x_m > 0; x_m++)
                {
                    if (map(x - x_m, y) > obs_thre)
                    {
                        if (dis > x_m * x_m)
                        {
                            dis = x_m * x_m;
                        }
                        break;
                    }
                }
                cells_[cellIndex(x, y)] = sqrt(dis) * map.metersPerCell();
            }
        }
    }
}

bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}

void ObstacleDistanceGrid::resetGrid(const OccupancyGrid &map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();

    // If the grid is already the correct size, nothing needs to be done
    if ((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }

    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();

    cells_.resize(width_ * height_);
}
