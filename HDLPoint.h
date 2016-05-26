#ifndef HDLPOINT_H_INCLUDED
#define HDLPOINT_H_INCLUDED

#include "Point.h"

// HDL meta point number
#define ANGLE_NUM 36000

typedef struct HDL_Point
{
    int x, y, z;
} HDLPoint_3D;

typedef struct LPoint
{
    int x;
    int y;
    int z;
    unsigned short dist;
    unsigned short rot;
    unsigned char i;
    unsigned char c;
} LPoint_t;

typedef struct PointSave
{
    unsigned short dist;
    unsigned short rot;
    unsigned char i;
    unsigned char c;
} PointSave_t;

typedef struct MetaLaserHdl {
	int pts_count;
	LPoint_t pts[HDL_MAX_POINT_NUMBER];
} MetaLaserHdl_t;

class HDLPoint: public Point
{
public:
    HDLPoint(int x = 0, int y = 0, int z = 0, unsigned short dist = 0, unsigned short rot = 0, unsigned char i = ' ', unsigned char c = ' ');
    //HDLPoint() {};
    ~HDLPoint() {};
    //int x;
	//int y;
	double z;
	unsigned short dist;
	unsigned short rot;
	unsigned char i;
	unsigned char c;
};

#endif // HDLPOINT_H_INCLUDED
