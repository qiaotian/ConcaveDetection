#include "HDLPoint.h"

HDLPoint::HDLPoint(int x, int y, int z, unsigned short dist, unsigned short rot, unsigned char i, unsigned char c)
{
//    x = p_x;
//    y = p_y;
//    z = p_z;
//    dist = p_dist;
//    rot = p_rot;
//    i = p_i;
//    c = p_c;

    this->x = x / 1000.0;
    this->y = y / 1000.0;
    this->z = z / 1000.0;
    this->dist = dist / 1000.0;
    this->rot = rot;
    this->i = i;
    this->c = c;
}
