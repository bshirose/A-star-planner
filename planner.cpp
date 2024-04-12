/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "planner.h"
#include <math.h>
#include <iostream>
#include <algorithm>
#include <thread>
#include <chrono>
#include <ctime>
struct CompareCellByF
{
    bool operator()(const cell *a, const cell *b)
    {
        return a->f > b->f; 
    }
};
bool ini{false};
int mini = 10000000;
int fx = -1;
int fy = -1;
int weight{1};
bool ini2{false};
bool screwcost{false};
int ind{0};
bool path_flag{false};

int wait{0};

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y - 1) * XSIZE + (X - 1))

#if !defined(MAX)
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

using namespace std;
#define NUMOFDIRS 8

Planner *p = new Planner;

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
    double *action_ptr)
{
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dY[NUMOFDIRS] = {-1, 0, 1, -1, 1, -1, 0, 1};

    int goalposeX = (int)target_traj[target_steps - 1];
    int goalposeY = (int)target_traj[target_steps - 1 + target_steps];

    int bestX = 0, bestY = 0;
    double olddisttotarget = (double)sqrt(((robotposeX - goalposeX) * (robotposeX - goalposeX) + (robotposeY - goalposeY) * (robotposeY - goalposeY)));
    double disttotarget;
    p->robot_x = robotposeX;
    p->robot_y = robotposeY;

    p->map_x = x_size;
    p->map_y = y_size;

    if (!ini)
    {
        p->target_x = goalposeX;
        p->target_y = goalposeY;
        p->calculate_h(collision_thresh, map);

        ini = true;
        p->robot_x = robotposeX;
        p->robot_y = robotposeY;
        int buff{0};
        int skip{5};
        if (x_size > 2500 || y_size > 2500)
        {
            skip = 50;
            buff = 200;
        }
        else if (x_size > 1500 || y_size > 1500)
        {
            skip = 25;
            buff = 100;  
        }
        else if (x_size > 300 || y_size > 300)
        {
            buff = 25;
        }
        else
        {
            skip = 1;
            buff = 5;
        }
        int tx{0};
        int ty{0};
        bool time_flag{false};

        for (int i = 20; i < target_steps; i++)
        {
            if (i % skip != 0)
            {
                continue;
            }

            tx = (int)target_traj[i];
            ty = (int)target_traj[target_steps + i];
            p->target_x = tx;
            p->target_y = ty;
            if (i % 20 == 0)
                p->update_h(collision_thresh, map);

            p->catch_target(collision_thresh, map);
            if (p->length + buff < i && mini > p->cost + std::max(1, p->min_cost) * (i - p->length))
            {
                mini = p->cost + (i - p->length);
                wait = i - p->length - buff - 10;
                fx = tx;
                fy = ty;
                if (p->min_cost == 0)
                {
                    break;
                }
            }
        }
        if (fx == -1)
        {
            fx = goalposeX;
            fy = goalposeY;
            screwcost = true;
        }
    }

    p->target_x = fx;
    p->target_y = fy;
    if (robotposeX == fx && robotposeY == fy)
    {
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        return;
    }
    // if (!ini2)
    // {
    //     p->update_h(collision_thresh, map);
    //     ini2 = true;
    // }
    if (!path_flag)
    {
        p->update_h(collision_thresh, map);
        std::cout << "Target will be caught at x = " << fx << " y = " << fy << endl;

        if (!screwcost)
        {
            p->catch_target(collision_thresh, map);
        }
        else
        {
            p->catch_targettime(collision_thresh, map);
        }
        path_flag = true;
    }
    if ((int)map[GETMAPINDEX(robotposeX, robotposeY, x_size, y_size)] < 2 && wait > 0)
    {
        wait = wait - 1;
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        return;
    }
    action_ptr[0] = p->coo[ind].first;
    action_ptr[1] = p->coo[ind].second;
    ind = ind + 1;

    return;
}

void Planner::update_h(int collision_threshold, double *map)
{
    float distance;
    for (int i = 1; i < map_x - 2; i++)
    {
        for (int j = 1; j < map_y - 2; j++)
        {
            distance = pow(i - target_x, 2) + pow(j - target_y, 2);
            g[i][j]->h = sqrt(distance);
            g[i][j]->g = 111111110;
            g[i][j]->visited = false;
            g[i][j]->parent = nullptr;
        }
    }
}

void Planner::reset_g(int collision_threshold, double *map)
{
    float distance;
    for (int i = 1; i < map_x - 2; i++)
    {
        for (int j = 1; j < map_y - 2; j++)
        {
            g[i][j]->g = 111111110;
            g[i][j]->visited = false;
            g[i][j]->parent = nullptr;
        }
    }
}

void Planner::calculate_h(int collision_threshold, double *map)
{
    float distance;

    for (int i = 0; i < map_x - 1; i++)
    {
        vector<cell *> row;

        for (int j = 0; j < map_y - 1; j++)
        {
            cell *block = new cell;
            block->c_x = i;
            block->c_y = j;
            distance = pow(i - target_x, 2) + pow(j - target_y, 2);
            block->h = sqrt(distance);
            if (i < 1 || j < 1 || i > map_x - 2 || j > map_y - 2)
            {
            }
            else if (int(map[GETMAPINDEX(i, j, map_x, map_y)]) >= collision_threshold)
            {
                block->collision = true;
            }
            else
            {
                block->path_cost = int(map[GETMAPINDEX(i, j, map_x, map_y)]);
            }

            row.push_back(block);
        }
        g.push_back(row);
    }
}
void Planner::catch_target(int collision_threshold, double *map)
{
    int i_x = robot_x;
    int i_y = robot_y;
    bool flag = false;
    priority_queue<cell *, vector<cell *>, CompareCellByF> priorityQueue;
    cell *current = g[robot_x][robot_y];
    current->c_x = robot_x;
    current->c_y = robot_y;
    current->visited = true;
    current->g = 0;
    current->f = current->g + current->h;
    priorityQueue.push(current);
    coo.clear();
    min_cost = 1000000;

    vector<cell *> expanded;

    while (priorityQueue.size() > 0)
    {
        current = priorityQueue.top();
        current->visited = true;
        if (current->c_x == target_x && current->c_y == target_y)
        {
            // std::cout << "target reached" << endl;
            cell *point = g[target_x][target_y];
            cost = 0;
            final_spot_cost = point->path_cost;

            while (point->parent != nullptr)
            {
                coo.push_back(make_pair(point->c_x, point->c_y));
                cost = cost + point->path_cost;
                if (point->path_cost < min_cost)
                {
                    min_cost = point->path_cost;
                }
                point = point->parent;
            }
            length = coo.size();
            std::reverse(coo.begin(), coo.end());
            int fx = target_x;
            int fy = target_y;
            break;
        }

        priorityQueue.pop();
        i_x = current->c_x;
        i_y = current->c_y;
        for (int i = 0; i < NUMOFDIRS; i++)
        {
            int new_x = i_x + dx[i];
            int new_y = i_y + dy[i];

            if (new_x < 2 || new_y < 2 || new_x > map_x - 3 || new_y > map_y - 3)
            {
                continue;
            }
            cell *neibour = g[new_x][new_y];
            if (!neibour->collision && !neibour->visited && (neibour->g > current->g + neibour->path_cost))
            {
                expanded.push_back(neibour);
                neibour->g = current->g + neibour->path_cost;

                neibour->f = neibour->g + weight * neibour->h;
                neibour->parent = current;
                priorityQueue.push(neibour);
            }
        }
    }

    for (int i = 0; i < expanded.size(); i++)
    {
        expanded[i]->g = 111111110;
        expanded[i]->visited = false;
        expanded[i]->parent = nullptr;
    }
}

void Planner::catch_targettime(int collision_threshold, double *map)
{

    int i_x = robot_x;
    int i_y = robot_y;
    bool flag = false;
    priority_queue<cell *, vector<cell *>, CompareCellByF> priorityQueue;
    cell *current = g[robot_x][robot_y];
    current->c_x = robot_x;
    current->c_y = robot_y;
    current->visited = true;
    current->g = 0;
    current->f = current->g + current->h;
    priorityQueue.push(current);
    coo.clear();
    while (priorityQueue.size() > 0)
    {
        current = priorityQueue.top();
        current->visited = true;
        if (current->c_x == target_x && current->c_y == target_y)
        {
            // std::cout << "target reached!!!!!!!!!!!" << endl;
            cell *point = g[target_x][target_y];
            cost = 0;
            final_spot_cost = point->path_cost;

            while (point->parent != nullptr)
            {
                coo.push_back(make_pair(point->c_x, point->c_y));
                cost = cost + point->path_cost;
                point = point->parent;
            }
            length = coo.size();
            std::reverse(coo.begin(), coo.end());
            int fx = target_x;
            int fy = target_y;
            break;
        }

        priorityQueue.pop();
        i_x = current->c_x;
        i_y = current->c_y;
        for (int i = 0; i < NUMOFDIRS; i++)
        {
            int new_x = i_x + dx[i];
            int new_y = i_y + dy[i];

            if (new_x < 2 || new_y < 2 || new_x > map_x - 3 || new_y > map_y - 3)
            {
                continue;
            }
            cell *neibour = g[new_x][new_y];
            if (!neibour->collision && !neibour->visited && (neibour->g > current->g + 1))
            {
                neibour->g = current->g + 1;

                neibour->f = neibour->g + weight * neibour->h;
                neibour->parent = current;
                priorityQueue.push(neibour);
            }
        }
    }
}

vector<int> Planner::path()
{

    vector<int> a;

    return a;
}