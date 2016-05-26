#ifndef CONFIGURATION_H_INCLUDED
#define CONFIGURATION_H_INCLUDED

//used for grid
const int adjacent_distance = 2;

//used for curves, lanes, curbs, guardrails
const double near_dist = 2.0;

//used for raster map
const double max_rou = 120.0;
const double max_theta = 360;
//const double max_length = 100.0;
//const double max_width = 100.0;

const double max_right = 250.0;
const double max_left = 250.0;
const double max_front = 250.0;
const double max_back = 250.0;

const double d_length = 0.2;
const double d_width  = 0.2;

const double max_length = max_front + max_back;
const double max_width = max_left + max_right;

const double pi = 3.14159;

//used for detect box
const double detect_right = 1.2;
const double detect_left = -1.2;
const double detect_front = 30.0;
const double detect_back = 0;

const double detect_width = detect_right - detect_left;
//used for scan HDL grid pos
//const double hdl_height_threshold = 0.20;

//used for adjust object width
const double ratio_length_width = 2.0;


//used for compute the correlation between two objects
//const double corr_a = 1.0;
//const double corr_b = 1.0;
//const double corr_c = 1.0;
//const double corr_d = 1.0;
const int threshold_corr = 20;

//used for track
const unsigned int min_belief = 3;
const unsigned int max_beleif = 10;

#endif // CONFIGURATION_H_INCLUDED
