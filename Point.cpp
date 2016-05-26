#include "Point.h"
#include "Configuration.h"

Point::Point(double x, double y, double dist)
{
    this->x = x;
    this->y = y;
    this->dist = dist;
    //cout << "Point's constructor called!" << endl;
    //cout << "x: " << x << " y:" << y << endl;
}

Point::Point(const Point& p)
{
    x = p.x;
    y = p.y;
    dist = p.dist;
}

Point::~Point()
{

}

void Point::setPoint(double x, double y)
{
    this->x = x;
    this->y = y;
    this->dist = sqrt(pow(x,2) + pow(y,2));
}

void Point::setX(double x)
{
    this->x = x;
    //this->dist = sqrt(pow(this->x,2) + pow(this->y,2));
}

void Point::setY(double y)
{
    this->y = y;
    //this->dist = sqrt(pow(this->x,2) + pow(this->y,2));
}

void Point::setDist(double dist)
{
    this->dist = dist;
}

double Point::getX() const
{
    return x;
}

double Point::getY() const
{
    return y;
}

double Point::getDist() const
{
    return dist;
}

bool Point::IsInDetectBox() const
{
    if((this->x < detect_right) && (this->x > detect_left) && (this->y < detect_front) && (this->y > detect_back))
        return true;
    else
        return false;
}

bool Point::IsInDetectBox()
{
    if((this->x < detect_right) && (this->x > detect_left) && (this->y < detect_front) && (this->y > detect_back))
        return true;
    else
        return false;
}

void Point::transToFrozen(module::Pose_t vehicle)
{
    double tx = this->x;
    double ty = this->y;
    this->x = cos(vehicle.eulr) * tx + sin(vehicle.eulr) * ty + vehicle.x;
    this->y = -sin(vehicle.eulr) * tx + cos(vehicle.eulr) * ty + vehicle.y;
}
