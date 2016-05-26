#ifndef POINT_H_INCLUDED
#define POINT_H_INCLUDED

#include <module/shm.h>
#include <module/module.h>
#include <cmath>
#include <iostream>
using namespace std;

class Point
{
public:
    Point(double x = 0, double y = 0, double dist = 0);
    Point(const Point& p);
    ~Point();

    void setDist(double dist);
    void setPoint(double x, double y);
    void setX(double x);
    void setY(double y);

    bool IsInDetectBox();
    bool IsInDetectBox() const;
    void transToFrozen(module::Pose_t vehicle);
    double getDist() const;
    double getX() const;
    double getY() const;
//private:
    double dist;
    double x, y;
};

#endif // POINT_H_INCLUDED
