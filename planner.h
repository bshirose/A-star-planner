#ifndef PLANNER_H
#define PLANNER_H
#include <vector>
#include <queue>


// map4 time cost 288583
//map4 path cost 70271
// map4 path cost 670


struct cell
{
    float h{1111110.0}, g{111111110.0}, f{111111110.0};
    int parent_x, parent_y;
    int c_x, c_y;
    bool collision{false}, visited{false};
    cell * parent=nullptr;
    int path_cost{0};
};


// Declare the plan function
void planner(
    double *map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    double *target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    double *action_ptr);
class Planner
{
public:
    std::vector<std::vector<cell *>> g;
    int x,y;
    int robot_x, robot_y, target_x, target_y, map_x, map_y;
    std::vector<std::pair<int,int>> coo;
    std::pair<int, int> start;
    int *map;
    int dx[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dy[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    int length = 0;
    float cost;
    int final_spot_cost;
    int min_cost{1000000};
    void catch_target(int collision_threshold, double *map);
    void catch_targettime(int collision_threshold, double *map);
    void calculate_h(int collision_threshold, double *map);
    void update_h(int collision_threshold, double *map);
    void reset_g(int collision_threshold, double *map);
    std::vector<int> path();
};
#endif // PLANNER_H