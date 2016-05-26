#pragma once
/*!
* \file Header.h
* \brief
*
*
* \author QiaoTian, qiaotian@me.com
* \version 1.0
* \date 2014-08-10
*/
#define _USE_MATH_DEFINES

#include <stdio.h>
#include <map>
#include <vector>
#include <list>
#include <string>
#include <iostream>
#include <fstream>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <opencv/cvaux.h>
#include <GL/glut.h>
#include <pthread.h>
//#include <windows.h>

#include <math.h>
#include <time.h>
//#include <tchar.h>
#include <stdlib.h>
//	#inlcude <module/shm.h>
//	#include <module/module.h>

using namespace std;

extern pthread_t g_thread_opengl;
extern pthread_rwlock_t g_lock_opengl;

#define M_PI       3.14159265358979323846

#define LASER_NUM 64
#define CLOUD_NUM 256000
#define ANGLE_NUM 36000

#define QT_INIT_MAXHEIGHT   -3000 // unit is mm
#define QT_INIT_MINHEIGHT	-1000 // unit is mm
#define QT_BOX_LENGTH		 5000 // unit is mm

#define HDL_LASER_NUMBER       64 
#define HDL_LASER_NUMBER_ROAD  55 // 
#define HDL_MAX_POINT_LASER    4000
#define HDL_MAX_POINT_NUMBER HDL_MAX_POINT_LASER * HDL_LASER_NUMBER
#define obj_height_thr         400

#define grid_size 200
#define local_grid_height 500
#define local_grid_width  500

#define freeze_grid_height 1000
#define freeze_grid_width  1000 

#define feature_point_threshold 0.6f

#define PT_TYPE_UNINDENTIFIED 0
#define PT_TYPE_ROAD 1
#define PT_TYPE_CURB 2

#define LEFT 0
#define RIGHT 1
#define SCANRANGE 50

#define FIRSTSEGMENT 1
#define SECONDSEGMENT 2
#define LEASTWIDTH 5.0
#define LANEWIDTH    3

typedef struct Grid {

	float _pForObstacle;
	float _pForNegativeObstacle;
	int _dotCount;					///< laser dot number in this grid
	int _maxHeight;					///< unit is mm
	int _minHeight;					///< unit is mm
	int _x;							///< unit is mm
	int _y;							///< unit is mm
}Grid, *pGrid;

typedef struct LPoint
{
	int x;
	int y;
	int z;
	unsigned short dist;
	unsigned short rot;  ///<
	unsigned char i;     ///< 
	unsigned char c;     ///< 
}LPoint_t;

typedef struct HDLScale{
	double x_min;
	double x_max;
	double y_min;
	double y_max;
	double x_scale;
	double y_scale;
}HDLScale_t;

typedef struct Point_laser
{
	int pt_count;
	LPoint_t pt[HDL_MAX_POINT_LASER];
	int road_type[HDL_MAX_POINT_LASER];// 0--can't pass 1--can pass 2--obstacle point
	int pt_type[HDL_MAX_POINT_LASER];
}Point_laser_t;

typedef struct Point_int
{
	int pt_count;
	double x_int[HDL_MAX_POINT_LASER];
	double y_int[HDL_MAX_POINT_LASER];
	double z_int[HDL_MAX_POINT_LASER];
	double x2_int[HDL_MAX_POINT_LASER];
	double xy_int[HDL_MAX_POINT_LASER];
	double y2_int[HDL_MAX_POINT_LASER];
}Point_int_t;

typedef struct Point3D
{
	int x;
	int y;
	int z;
}Point3D_t;

typedef struct Point3Df
{
	double x;
	double y;
	double z;
}Point3Df_t;

typedef struct PointSave
{
	int x;
	int y;
	int z;

	unsigned short dist;
	unsigned short rot;
	unsigned char i;
	unsigned char c;
}PointSave_t;

typedef struct MyColor
{
	unsigned int r;
	unsigned int g;
	unsigned int b;
}MyColor_t;

typedef struct Pose
{
	double x;
	double y;
	double eulr;
}Pose_t;

enum FittingMode{
	LINE_FITTING,
	POLY_FITTING
};

enum RunMode {
	ONLINE,
	OFFLINE
};

enum PlaneFittingMode {
	PLANE_ON,
	PLANE_OFF
};
