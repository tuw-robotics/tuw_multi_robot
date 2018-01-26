#ifndef POINT_H
#define POINT_H

#include <vector>
#include <eigen3/Eigen/Dense>

using Point = Eigen::Vector2d;

typedef struct Potential_Point_t
{
    Point point;
    float potential;
    Potential_Point_t(float _x, float _y, float _potential) : point(_x, _y), potential(_potential) {};
    Potential_Point_t(): Potential_Point_t(0, 0, 0) {};
} Potential_Point;


typedef struct PathPrecondition_t
{
    int robot;
    int stepCondition;
} PathPrecondition;

typedef struct PathSegment_t
{
    int segId;
    std::vector<PathPrecondition> preconditions;
    Point  start;
    Point  end;
} PathSegment;

#endif
