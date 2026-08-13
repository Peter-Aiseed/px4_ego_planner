#include "path_searching/dyn_a_star.h"

using namespace std;
using namespace Eigen;

AStar::~AStar()
{
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            for (int k = 0; k < POOL_SIZE_(2); k++)
                delete GridNodeMap_[i][j][k];
}

void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
{
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;

    GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                GridNodeMap_[i][j][k] = new GridNode;
            }
        }
    }

    grid_map_ = occ_map;
}

double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i &start_idx, Vector3i &end_idx)
{
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    if (checkOccupancy(Index2Coord(start_idx)))
    {
        //ROS_WARN("Start point is insdide an obstacle.");
        do
        {
            start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;
            if (!Coord2Index(start_pt, start_idx))
                return false;
        } while (checkOccupancy(Index2Coord(start_idx)));
    }

    if (checkOccupancy(Index2Coord(end_idx)))
    {
        //ROS_WARN("End point is insdide an obstacle.");
        do
        {
            end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;
            if (!Coord2Index(end_pt, end_idx))
                return false;
        } while (checkOccupancy(Index2Coord(end_idx)));
    }

    return true;
}

bool AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt)
{
    ros::Time time_1 = ros::Time::now();
    ++rounds_;

    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;

    // --- 1. Clamp End Goal into Active Rolling Map Window ---
    if (!grid_map_->isInMap(end_pt)) {
      Eigen::Vector3d dir = (end_pt - start_pt).normalized();
      Eigen::Vector3d temp_pt = end_pt;
      
      // Step back toward start_pt until the point falls inside the local rolling window
      while (!grid_map_->isInMap(temp_pt) && (temp_pt - start_pt).norm() > 0.5) {
        temp_pt -= dir * 0.2; // Step back 20cm
      }
      end_pt = temp_pt;
    }

    center_ = (start_pt + end_pt) / 2;

    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        ROS_ERROR("Unable to handle the initial or end point, force return!");
        return false;
    }

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    // --- 2. Ring Buffer Index Conversion for GridNodeMap Pool ---
    int start_rx = (start_idx(0) % POOL_SIZE_(0) + POOL_SIZE_(0)) % POOL_SIZE_(0);
    int start_ry = (start_idx(1) % POOL_SIZE_(1) + POOL_SIZE_(1)) % POOL_SIZE_(1);
    int start_rz = (start_idx(2) % POOL_SIZE_(2) + POOL_SIZE_(2)) % POOL_SIZE_(2);

    int end_rx = (end_idx(0) % POOL_SIZE_(0) + POOL_SIZE_(0)) % POOL_SIZE_(0);
    int end_ry = (end_idx(1) % POOL_SIZE_(1) + POOL_SIZE_(1)) % POOL_SIZE_(1);
    int end_rz = (end_idx(2) % POOL_SIZE_(2) + POOL_SIZE_(2)) % POOL_SIZE_(2);

    // Retrieve nodes using ring-buffered pool addresses
    GridNodePtr startPtr = GridNodeMap_[start_rx][start_ry][start_rz];
    GridNodePtr endPtr   = GridNodeMap_[end_rx][end_ry][end_rz];

    // Ensure the node metadata matches the current target coordinates
    startPtr->index = start_idx;
    endPtr->index   = end_idx;

    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET; //put start node in open set
    startPtr->cameFrom = NULL;
    openSet_.push(startPtr); //put start in open set

    endPtr->index = end_idx;

    double tentative_gScore;

    int num_iter = 0;
    while (!openSet_.empty())
    {
        num_iter++;
        current = openSet_.top();
        openSet_.pop();

        // if ( num_iter < 10000 )
        //     cout << "current=" << current->index.transpose() << endl;

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
        {
            // ros::Time time_2 = ros::Time::now();
            // printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, (time_2 - time_1).toSec()*1000);
            // if((time_2 - time_1).toSec() > 0.1)
            //     ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
            gridPath_ = retrievePath(current);
            return true;
        }
        current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;

                    Vector3i neighborIdx;
                    neighborIdx(0) = (current->index)(0) + dx;
                    neighborIdx(1) = (current->index)(1) + dy;
                    neighborIdx(2) = (current->index)(2) + dz;

                    if (!grid_map_->isInMap(neighborIdx))
                    {
                        continue;
                    }

                    int ring_x = (neighborIdx(0) % POOL_SIZE_(0) + POOL_SIZE_(0)) % POOL_SIZE_(0);
                    int ring_y = (neighborIdx(1) % POOL_SIZE_(1) + POOL_SIZE_(1)) % POOL_SIZE_(1);
                    int ring_z = (neighborIdx(2) % POOL_SIZE_(2) + POOL_SIZE_(2)) % POOL_SIZE_(2);

                    neighborPtr = GridNodeMap_[ring_x][ring_y][ring_z];
                    neighborPtr->index = neighborIdx;

                    bool flag_explored = (neighborPtr->rounds == rounds_) && (neighborPtr->index == neighborIdx);

                    if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                    {
                        continue; //in closed set.
                    }

                    neighborPtr->rounds = rounds_;

                    if (checkOccupancy(Index2Coord(neighborPtr->index)))
                    {
                        continue;
                    }

                    double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                    tentative_gScore = current->gScore + static_cost;

                    if (!flag_explored)
                    {
                        //discover a new node
                        neighborPtr->state = GridNode::OPENSET;
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        openSet_.push(neighborPtr); //put neighbor in open set and record it.
                    }
                    else if (tentative_gScore < neighborPtr->gScore)
                    { //in open set and need update
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        openSet_.push(neighborPtr); //put neighbor in open set and record it.
                    }
                }
        ros::Time time_2 = ros::Time::now();
        if ((time_2 - time_1).toSec() > 0.2)
        {
            ROS_WARN("Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
            return false;
        }
    }

    ros::Time time_2 = ros::Time::now();

    if ((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).toSec(), num_iter);

    return false;
}

vector<Vector3d> AStar::getPath()
{
    vector<Vector3d> path;

    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    reverse(path.begin(), path.end());
    return path;
}
