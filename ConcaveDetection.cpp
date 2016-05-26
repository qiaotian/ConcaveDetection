#include "ConcaveDetector.h"

using namespace std;

ConcaveDetector::ConcaveDetector()
{
	m_hdldata		= NULL;
	m_cloud			= NULL;
	m_cloud_saved	= NULL;
	m_point			= NULL;
	m_point_inbox	= NULL;
	m_cloud			= NULL;
	m_cos_raw		= NULL;
	m_sin_raw		= NULL;
}

ConcaveDetector::~ConcaveDetector()
{
	if (m_hdldata)
	{
		fclose(m_hdldata);
		m_hdldata = NULL;
	}
	if (m_cloud)
	{
		delete[] m_cloud;
		m_cloud = NULL;
	}
	if (m_cloud_saved)
	{
		delete[] m_cloud_saved;
		m_cloud_saved = NULL;
	}
	if (m_point) {
		delete[] m_point;
		m_point = NULL;
	}
	if (m_point_inbox){
		delete[] m_point_inbox;
		m_point_inbox = NULL;
	}
	if (m_cos_raw)
	{
		delete[] m_cos_raw;
		m_cos_raw = NULL;
	}
	if (m_sin_raw)
	{
		delete[] m_sin_raw;
		m_sin_raw = NULL;
	}
}

bool ConcaveDetector::Initialize() {
	m_cloud       = new LPoint_t[HDL_MAX_POINT_NUMBER];
	m_cloud_saved = new PointSave_t[HDL_MAX_POINT_NUMBER];
	m_point       = new Point_laser_t[HDL_LASER_NUMBER];
	m_point_inbox = new Point_laser_t[HDL_LASER_NUMBER];
	m_cloud_count = 0;

	initLocalGrids();
	return true;
}

bool ConcaveDetector::Initialize(const char* hdlfilepath){
	m_cloud       = new LPoint_t[HDL_MAX_POINT_NUMBER];
	m_cloud_saved = new PointSave_t[HDL_MAX_POINT_NUMBER];
	m_point       = new Point_laser_t[HDL_LASER_NUMBER];
	m_point_inbox = new Point_laser_t[HDL_LASER_NUMBER];
	m_cloud_count = 0;
	m_cos_raw     = new float[ANGLE_NUM];             // ANGLE_NUM = 36000
	m_sin_raw     = new float[ANGLE_NUM];
	int temp[] = { 39, 40, 43, 44, 33, 34, 37, 38, 41, 42,
		47, 48, 51, 52, 55, 56, 45, 46, 49, 50,
		53, 54, 59, 60, 63, 64, 35, 36, 57, 58,
		61, 62, 7, 8, 11, 12, 1, 2, 5, 6,
		9, 10, 15, 16, 19, 20, 23, 24, 13, 14,
		17, 18, 21, 22, 27, 28, 31, 32, 3, 4,
		25, 26, 29, 30
	};
	for (int i = 0; i < HDL_LASER_NUMBER; i++){
		m_laser_index[i] = temp[i];
	}

	if (!load_laser_color("colormatrix.txt")) return false;
	if (!load_laser_info("new_xml.txt")) return false;
	if ((m_hdldata = fopen(hdlfilepath, "rb")) == NULL)
	{
		cout << "read hdl file " << hdlfilepath << " error!" << endl;
		return false;
	}

	initLocalGrids();
	return true;
}

bool ConcaveDetector::load_laser_color(const char* colorpath)
{
	int index;
	ifstream fdb(colorpath);
	if (!fdb)
	{
		cout << "read color matrix file error!" << endl;
		return false;
	}

	for (int i = 0; i < HDL_LASER_NUMBER; i++)
	{
		fdb >> index >> m_lasercolor[i].r >> m_lasercolor[i].g >> m_lasercolor[i].b;
	}
	fdb.close();

	return true;
}

bool ConcaveDetector::load_laser_info(const char* data_path)
{
	//the dat file is converted from db.xml, only have the array info
	//used parse_xml_db solution to convert xml to dat file
	ifstream fdb(data_path);
	if (!fdb)
	{
		cout << "read laser info file error!" << endl;
		return false;
	}

	for (int i = 0; i < HDL_LASER_NUMBER; i++)
	{
		fdb >> m_rot[i] >> m_vert[i] >> m_dist[i] >> m_z_off[i] >> m_x_off[i] >> m_min_i[i] >> m_max_i[i] >> m_distX[i] >> m_distY[i] >> m_f_d[i] >> m_f_s[i];
	}
	fdb.close();

	//regulate unit to mm
	for (int i = 0; i < HDL_LASER_NUMBER; i++)
	{
		m_dist[i] *= 10;
		m_z_off[i] *= 10;
		m_x_off[i] *= 10;
		m_distX[i] *= 10;
		m_distY[i] *= 10;
	}

	//pre-processing sin, cos array
	for (int i = 0; i < HDL_LASER_NUMBER; i++)
	{
		m_cos_rot[i] = (float)cos(m_rot[i] / 180.0f * M_PI);
		m_sin_rot[i] = (float)sin(m_rot[i] / 180.0f * M_PI);
		m_cos_vert[i] = (float)cos(m_vert[i] / 180.0f * M_PI);
		m_sin_vert[i] = (float)sin(m_vert[i] / 180.0f * M_PI);
	}

	for (int i = 0; i < ANGLE_NUM; i++)
	{
		m_cos_raw[i] = (float)cos(i / 18000.0f * M_PI);
		m_sin_raw[i] = (float)sin(i / 18000.0f * M_PI);
	}

	return true;
}

void ConcaveDetector::initLocalGrids(){
	for (int i = 1; i < local_grid_height; i++) {
		for (int j = 1; j < local_grid_width; j++)
		{
			_localGrids[i][j]._x = 0;
			_localGrids[i][j]._y = 0;
			_localGrids[i][j]._maxHeight = -10000;
			_localGrids[i][j]._minHeight = 10000;
			_localGrids[i][j]._dotCount  = 0;
			_localGrids[i][j]._pForObstacle = 0.5;			// initialize the empty cell with 0.5
			_localGrids[i][j]._pForNegativeObstacle = 0.5;
		}
	}
}

void ConcaveDetector::initGlobalGrids(){
	for (int i = 1; i < freeze_grid_height; i++) {
		for (int j = 1; j < freeze_grid_width; j++)
		{
			_localGrids[i][j]._x = 0;
			_localGrids[i][j]._y = 0;
			_localGrids[i][j]._maxHeight = -10000;
			_localGrids[i][j]._minHeight = 10000;
			_localGrids[i][j]._dotCount = 0;
			_localGrids[i][j]._pForObstacle = 0.5;			// initialize the empty cell with 0.5
			_localGrids[i][j]._pForNegativeObstacle = 0.5;
		}
	}
}

bool ConcaveDetector::updateLocalGridsUsingRelativeHeight(){
	initLocalGrids();
	// Update Max and Min Height of Grid
	for (int i = 0; i < HDL_LASER_NUMBER_ROAD; ++i) {
		int count = m_point[i].pt_count;
		for (int j = 0; j < count; ++j) {
			LPoint_t *tmp_pt = &m_point[i].pt[j]; // unit is mm
			if (tmp_pt->x < 50000 && tmp_pt->x > -50000 && tmp_pt->y < 50000 && tmp_pt->y >-50000)
			{
				// calculate the grid that it belongs to and update localGrids[][]
				int col = (int)floor(tmp_pt->x / 200 + 250);
				int row = (int)floor(tmp_pt->y / 200 + 250);
				_localGrids[row][col]._dotCount++;
				// update the maxheight and minheight of the grid
				if (tmp_pt->z > _localGrids[row][col]._maxHeight)
					_localGrids[row][col]._maxHeight = tmp_pt->z;
				if (tmp_pt->z < _localGrids[row][col]._minHeight)
					_localGrids[row][col]._minHeight = tmp_pt->z;
			}
		}
	}
	/*int number = 0;
	for (int i = 0; i < local_grid_width; ++i) {
		for (int j = 0; j < local_grid_height; ++j) {
			number += _localGrids[i][j]._dotCount;
		}
	}*/
	// Update All The Probability of Grids
	for (int row = 0; row < local_grid_width; row++)
	{
		for (int col = 0; col < local_grid_height; col++)
		{
			if (_localGrids[row][col]._dotCount > 1)
			{
				// update pForObstacle
				if (_localGrids[row][col]._maxHeight - _localGrids[row][col]._minHeight < obj_height_thr)
					_localGrids[row][col]._pForObstacle = 0.3;
				else {
					_localGrids[row][col]._pForObstacle = 0.8;
				}
				//featurePoint.push_back(Point2f((col - 250) * 200, (row - 250) * 200));
				// update pForNegativeObstacle
				_localGrids[row][col]._pForNegativeObstacle	= 0.3;
			}
			else
			{
				_localGrids[row][col]._pForObstacle = 0.5;
			}
		}
	}
	return true;
}

bool ConcaveDetector::updateLocalGridsUsingAbsoluteHeight(){
	initLocalGrids();
	/// 6.Update Max and Min Height of Grids
	for (int i = 0; i < HDL_LASER_NUMBER_ROAD; ++i) {
		int count = m_point[i].pt_count;
		for (int j = 0; j < count; ++j) {
			LPoint_t *tmp_pt = &m_point[i].pt[j]; // unit is mm
			if (tmp_pt->x < 50000 && tmp_pt->x > -50000 && tmp_pt->y < 50000 && tmp_pt->y >-50000)
			{
				// calculate the grid that it belongs to and update localGrids[][]
				int col = (int)floor(tmp_pt->x / 200 + 250);
				int row = (int)floor(tmp_pt->y / 200 + 250);
				_localGrids[row][col]._dotCount++;
				// update the maxheight and minheight of the grid
				if (tmp_pt->z > _localGrids[row][col]._maxHeight)
					_localGrids[row][col]._maxHeight = tmp_pt->z;
				if (tmp_pt->z < _localGrids[row][col]._minHeight)
					_localGrids[row][col]._minHeight = tmp_pt->z;
			}
		}
	}
	cout << "6.Update max_height, min_height in Local Grids Succeed!" << endl;
	// Update Probility of Grids
	for (int row = 0; row < local_grid_width; row++)
	{
		for (int col = 0; col < local_grid_height; col++)
		{
			if (_localGrids[row][col]._dotCount > 0)
			{
				// update pForObstacle
				if (_localGrids[row][col]._maxHeight + 2000 < obj_height_thr)
					_localGrids[row][col]._pForObstacle = 0.3;
				else {
					_localGrids[row][col]._pForObstacle = 0.8;// positive obsatacle
				}
				_localGrids[row][col]._pForNegativeObstacle = 0.3;
			}
			else
			{
				_localGrids[row][col]._pForObstacle = 0.5;
			}
		}
	}
	cout << "6.Update probility in Local Grids Success!" << endl;
	return true;
}

bool ConcaveDetector::updateGlobalGrid(){
	for (int i = 0; i < local_grid_height; i++) {
		for (int j = 0; j < local_grid_width; j++)
		{
			/*_localGrids[i][j]._x = 0;
			_localGrids[i][j]._y = 0;
			_localGrids[i][j]._maxHeight = -10000;
			_localGrids[i][j]._minHeight = 10000;
			_localGrids[i][j]._dotCount = 0;
			_localGrids[i][j]._pForObstacle = 0.5;			// initialize the empty cell with 0.5
			_localGrids[i][j]._pForNegativeObstacle = 0.5;*/
			if (_localGrids[i][j]._pForNegativeObstacle == 0.8)
			{
				double src_x = (j - 250) * 200;
				double src_y = (i - 250) * 200;
				double delta_theta = current_vehicle.theta - freezed_vehicle.theta;
				double delta_x = current_vehicle.x - freezed_vehicle.x;
				double delta_y = current_vehicle.y - freezed_vehicle.y;
				double dest_x = src_x*cos(delta_theta) + src_y*sin(delta_theta) + delta_x;
				double dest_y = src_y*cos(delta_theta) - src_x*sin(delta_theta) + delta_y;

				int col = floor(dest_x / 200) + 500;
				int row = floor(dest_y / 200) + 500;
				// update probability
				//_globalGrids[row][col]._pForNegativeObstacle = xxxx;
			}
		}
	}
	return true;
}

bool ConcaveDetector::ReadDataFromShm(){
	/*m_data_hdl_points.type = module::MetaData::META_LASER_HDL;
	module::shm::SHARED_OBJECTS.GetMetaData(&m_data_hdl_points);
	m_hdl_points = m_data_hdl_points.value.v_laserHdl;
	m_cloud_count = m_data_hdl_points.value.v_laserHdl.pts_count;
	//m_cloud_count = 1;
	for (int ii = 0; ii < m_cloud_count; ++ii){
		m_cloud[ii].x = m_hdl_points.pts[ii].x;
		m_cloud[ii].y = m_hdl_points.pts[ii].y;
		m_cloud[ii].z = m_hdl_points.pts[ii].z;
		m_cloud[ii].dist = m_hdl_points.pts[ii].dist;
		m_cloud[ii].i = m_hdl_points.pts[ii].i;
		m_cloud[ii].c = m_hdl_points.pts[ii].c;
		m_cloud[ii].rot = m_hdl_points.pts[ii].rot;
		//m_cloud[ii].x = 0;
		//m_cloud[ii].y = 1;
		//m_cloud[ii].z = 1;
		//m_cloud[ii].dist = 1;
		//m_cloud[ii].i = 1;
		//m_cloud[ii].c = 2;
		//m_cloud[ii].rot = 1;
	}*/
	return true;
}

bool ConcaveDetector::ReadDataFromFile(){
	unsigned short rot;
	unsigned char c;

	if ((int)fread(&m_cloud_count, sizeof(int), 1, m_hdldata) < 1)
		return false;
	if ((int)fread(m_cloud_saved, sizeof(PointSave_t), m_cloud_count, m_hdldata) < m_cloud_count)
		return false;

	for (int i = 0; i < m_cloud_count; i++)
	{
		m_cloud[i].i    = m_cloud_saved[i].i;
		m_cloud[i].dist = m_cloud_saved[i].dist;
		m_cloud[i].c    = m_cloud_saved[i].c;
		m_cloud[i].rot  = m_cloud_saved[i].rot;
		c = m_cloud[i].c;
		rot = m_cloud[i].rot;

		float cos_phi = m_cos_vert[c];
		float sin_phi = m_sin_vert[c];
		float cos_theta = m_cos_raw[rot] * m_cos_rot[c] + m_sin_raw[rot] * m_sin_rot[c];
		float sin_theta = m_sin_raw[rot] * m_cos_rot[c] - m_cos_raw[rot] * m_sin_rot[c];
		float r1 = m_cloud_saved[i].dist * 2.0f;
		float r = r1 + m_dist[c];

		float rxy = r * cos_phi;
		float xx = abs(rxy * sin_theta - m_x_off[c] * cos_theta);
		float yy = abs(rxy * cos_theta + m_x_off[c] * sin_theta);

		float rx = (m_dist[c] - m_distX[c]) * (xx/22640.0f - 0.106007f) + m_distX[c];
		float ry = (m_dist[c] - m_distY[c]) * (yy/23110.0f - 0.083514f) + m_distY[c];

		//x:
		r = r1 + rx;
		rxy = r * cos_phi;
		int x = (int)(rxy * sin_theta - m_x_off[c] * cos_theta);

		//y:
		r = r1 + ry;
		rxy = r * cos_phi;
		int y = (int)(rxy * cos_theta + m_x_off[c] * sin_theta);

		//z:
		r = r1 + m_dist[c];
		int z = (int)(r * sin_phi + m_z_off[c]);

		m_cloud[i].x = x;
		m_cloud[i].y = y;
		m_cloud[i].z = z;
	}

	return true;
}

bool ConcaveDetector::ProcessArray(){
	// 1.store origin points into two dimentional array
	int c, count[HDL_LASER_NUMBER];
	for (int i = 0; i < HDL_LASER_NUMBER; i++){
		count[i] = 0;
	}
	for (int i = 0; i < m_cloud_count; i++){
		c = m_cloud[i].c;
		m_point[c].pt[count[c]].x = m_cloud[i].x;
		m_point[c].pt[count[c]].y = m_cloud[i].y;
		m_point[c].pt[count[c]].z = m_cloud[i].z;
		m_point[c].road_type[count[c]] = 0;
		m_point[c].pt[count[c]].dist = m_cloud[i].dist;
		m_point[c].pt[count[c]].i = m_cloud[i].i;
		m_point[c].pt[count[c]].c = m_cloud[i].c;
		m_point[c].pt[count[c]].rot = 9000 - m_cloud[i].rot - int(laser_angle[c] * 100);

		if (m_point[c].pt[count[c]].rot < 0)
		{
			m_point[c].pt[count[c]].rot += 36000;
		}

		//m_point[c].pt[count[c]].rot = m_cloud[i].rot;
		//m_point[c].is_road[count[c]] = false;
		//m_point[c].pt[count[c]].index = i ;
		count[c]++;
	}
	// 2.update the number of points in every line
	for (int i = 0; i < HDL_LASER_NUMBER; i++)
	{
		m_point[i].pt_count = count[i];
	}

	// 3.sort the 64 lasers from near to far
	Point_laser* pl = new Point_laser[HDL_LASER_NUMBER];
	for (int i = 0; i < HDL_LASER_NUMBER; i++)
	{
		pl[i] = m_point[m_laser_index[i] - 1];
	}
	for (int i = 0; i < HDL_LASER_NUMBER; i++)
	{
		m_point[i] = pl[i];
	}
	delete[] pl;
	return true;
}

CvScalar lineRansac(const vector<CvPoint3D32f>& pt, float param_tol) {
	if (pt.size() == 295)
	{
		cout << "" << endl;
	}

	CvMat *matA = NULL, *matB = NULL, *matX = NULL, *matX_opt = NULL;
	CvMat *matA_full = NULL, *matB_full = NULL, *matX_full = NULL, *matDiff_full = NULL;
	CvMat *matA_filtered = NULL, *matB_filtered = NULL;

	const int    RANSAC_LOOP = 50;
	const double RANSAC_TOL = param_tol;
	const double RANSAC_TOL_STRICT = param_tol/2.0;
	const double RANSAC_LEAST_SIZE = 0.5;

	int counter = 0, max_counter = 0;
	int s[3];
	double x, y, a, b, c, y_min, y_max, y_cur;
	double belief;
	vector<int> filtered_index;
	CvScalar param = cvScalar(0);

	if ((int)pt.size() <= 2) //if ((int)pt.size() < 10)          // tan 20131126 if ((int)pt.size() < 10)
		return cvScalar(0);

	matA = cvCreateMat(2, 2, CV_32FC1);
	matB = cvCreateMat(2, 1, CV_32FC1);
	matX = cvCreateMat(2, 1, CV_32FC1);
	matX_opt = cvCreateMat(2, 1, CV_32FC1);
	matA_full = cvCreateMat((int)pt.size(), 2, CV_32FC1);
	matB_full = cvCreateMat((int)pt.size(), 1, CV_32FC1);
	matX_full = cvCreateMat((int)pt.size(), 1, CV_32FC1);
	matDiff_full = cvCreateMat((int)pt.size(), 1, CV_32FC1);

	for (int i = 0; i < (int)pt.size(); i++)
	{
		x = (double)pt[i].x * 1000;
		y = (double)pt[i].y / 1000;
		//cvmSet(matA_full, i, 0, y * y);
		cvmSet(matA_full, i, 0, y);
		cvmSet(matA_full, i, 1, 1);
		cvmSet(matB_full, i, 0, x);
	}

	//cvSolve(matA_full, matB_full, matX_opt, CV_SVD);

	//a = cvmGet(matX_opt, 0, 0);
	//b = cvmGet(matX_opt, 1, 0);
	//c = cvmGet(matX_opt, 2, 0);

	//param = cvScalar(a, b, c, 1);

	for (int cycle_counter = 0; cycle_counter < RANSAC_LOOP; cycle_counter++)
	{
		s[0] = (int)((double)rand() / RAND_MAX * ((int)pt.size() - 1));
		do
		{
			s[1] = (int)((double)rand() / RAND_MAX * ((int)pt.size() - 1));
		} while (s[0] == s[1]);
		//do
		//{
		//s[2] = (int)((double)rand() / RAND_MAX * ((int)pt.size() - 1));
		//} while (s[2] == s[0] || s[2] == s[1]);

		for (int i = 0; i < 2; i++)
		{
			x = (double)pt[s[i]].x * 1000;
			y = (double)pt[s[i]].y / 1000;
			//cvmSet(matA, i, 0, y * y);
			cvmSet(matA, i, 0, y);
			cvmSet(matA, i, 1, 1);
			cvmSet(matB, i, 0, x);
		}

		cvSolve(matA, matB, matX);
		cvGEMM(matA_full, matX, 1, matB_full, -1, matDiff_full);

		counter = 0;
		for (int i = 0; i < (int)pt.size(); i++)
		{
			if (fabs(cvmGet(matDiff_full, i, 0)) < RANSAC_TOL)
				counter++;
		}

		if (counter > max_counter)
		{
			cvCopy(matX, matX_opt);
			max_counter = counter;
			filtered_index.clear();
			for (int i = 0; i < (int)pt.size(); i++)
			{
				if (fabs(cvmGet(matDiff_full, i, 0)) < RANSAC_TOL_STRICT)
					filtered_index.push_back(i);
			}
		}
	}

	if ((int)filtered_index.size() >= 2 && (int)filtered_index.size() > (int)(RANSAC_LEAST_SIZE * max_counter))
	{
		matA_filtered = cvCreateMat((int)filtered_index.size(), 2, CV_32FC1);
		matB_filtered = cvCreateMat((int)filtered_index.size(), 1, CV_32FC1);

		y_min = y_max = pt[filtered_index[0]].y;
		for (int i = 0; i < (int)filtered_index.size(); i++)
		{
			y_cur = pt[filtered_index[i]].y;
			if (y_cur > y_max)
				y_max = y_cur;
			if (y_cur < y_min)
				y_min = y_cur;

			cvmSet(matA_filtered, i, 0, cvmGet(matA_full, filtered_index[i], 0));
			cvmSet(matA_filtered, i, 1, cvmGet(matA_full, filtered_index[i], 1));
			//cvmSet(matA_filtered, i, 2, cvmGet(matA_full, filtered_index[i], 2));
			cvmSet(matB_filtered, i, 0, cvmGet(matB_full, filtered_index[i], 0));
		}
		cvSolve(matA_filtered, matB_filtered, matX_opt, CV_SVD);

		cvReleaseMat(&matA_filtered);
		cvReleaseMat(&matB_filtered);

		a = (double)cvmGet(matX_opt, 0, 0);
		b = (double)cvmGet(matX_opt, 1, 0);
		//c = (float) cvmGet(matX_opt, 2, 0);
		c = 0;
		belief = (double)filtered_index.size() / max_counter;

		//if (fabs(a) <= 0.05 && fabs(b) <= 0.5 && (y_max - y_min) > 1.0)//tan 20131126 10)
		if (1)//tian 20140809
			param = cvScalar(a, b, c, belief);
		else
		{
			param = cvScalar(0);

			//cout <<" massege 1 " << " a = " << a << " b = " << b  <<  " c = " << c <<" y_max = " << y_max<< "  y_min=" << y_min << endl;
		}
	}
	else
	{
		param = cvScalar(0);
		//cout << " message 2 " <<  "size =" << filtered_index.size() << " least " << RANSAC_LEAST_SIZE * max_counter << endl;
	}

	cvReleaseMat(&matA);
	cvReleaseMat(&matB);
	cvReleaseMat(&matX);
	cvReleaseMat(&matX_opt);
	cvReleaseMat(&matA_full);
	cvReleaseMat(&matB_full);
	cvReleaseMat(&matX_full);
	cvReleaseMat(&matDiff_full);

	return param;
}

CvScalar polyRansac(const vector<CvPoint3D32f>& pt) {
	if (pt.size() == 295)
	{
		cout << "" << endl;
	}

	CvMat *matA = NULL, *matB = NULL, *matX = NULL, *matX_opt = NULL;
	CvMat *matA_full = NULL, *matB_full = NULL, *matX_full = NULL, *matDiff_full = NULL;
	CvMat *matA_filtered = NULL, *matB_filtered = NULL;

	const int RANSAC_LOOP = 50;
	const float RANSAC_TOL = 0.5;
	const float RANSAC_TOL_STRICT = 0.35;
	const float RANSAC_LEAST_SIZE = 0.5;

	int counter = 0, max_counter = 0;
	int s[3];
	float x, y, a, b, c, y_min, y_max, y_cur;
	double belief;
	vector<int> filtered_index;
	CvScalar param = cvScalar(0);

	if ((int)pt.size() <= 3) //if ((int)pt.size() < 10)
		return cvScalar(0);

	matA = cvCreateMat(3, 3, CV_32FC1);
	matB = cvCreateMat(3, 1, CV_32FC1);
	matX = cvCreateMat(3, 1, CV_32FC1);
	matX_opt = cvCreateMat(3, 1, CV_32FC1);
	matA_full = cvCreateMat((int)pt.size(), 3, CV_32FC1);
	matB_full = cvCreateMat((int)pt.size(), 1, CV_32FC1);
	matX_full = cvCreateMat((int)pt.size(), 1, CV_32FC1);
	matDiff_full = cvCreateMat((int)pt.size(), 1, CV_32FC1);

	for (int i = 0; i < (int)pt.size(); i++)
	{
		x = (float)pt[i].x / 1000;
		y = (float)pt[i].y / 1000;
		cvmSet(matA_full, i, 0, y * y);
		cvmSet(matA_full, i, 1, y);
		cvmSet(matA_full, i, 2, 1);
		cvmSet(matB_full, i, 0, x);
	}

	//cvSolve(matA_full, matB_full, matX_opt, CV_SVD);

	//a = cvmGet(matX_opt, 0, 0);
	//b = cvmGet(matX_opt, 1, 0);
	//c = cvmGet(matX_opt, 2, 0);

	//param = cvScalar(a, b, c, 1);

	for (int cycle_counter = 0; cycle_counter < RANSAC_LOOP; cycle_counter++)
	{
		s[0] = (int)((double)rand() / RAND_MAX * ((int)pt.size() - 1));
		do
		{
			s[1] = (int)((double)rand() / RAND_MAX * ((int)pt.size() - 1));
		} while (s[0] == s[1]);
		do
		{
			s[2] = (int)((double)rand() / RAND_MAX * ((int)pt.size() - 1));
		} while (s[2] == s[0] || s[2] == s[1]);

		for (int i = 0; i < 3; i++)
		{
			x = (float)pt[s[i]].x / 1000;
			y = (float)pt[s[i]].y / 1000;
			cvmSet(matA, i, 0, y * y);
			cvmSet(matA, i, 1, y);
			cvmSet(matA, i, 2, 1);
			cvmSet(matB, i, 0, x);
		}

		cvSolve(matA, matB, matX);
		cvGEMM(matA_full, matX, 1, matB_full, -1, matDiff_full);

		counter = 0;
		for (int i = 0; i < (int)pt.size(); i++)
		{
			if (fabs(cvmGet(matDiff_full, i, 0)) < RANSAC_TOL)
				counter++;
		}

		if (counter > max_counter)
		{
			cvCopy(matX, matX_opt);
			max_counter = counter;
			filtered_index.clear();
			for (int i = 0; i < (int)pt.size(); i++)
			{
				if (fabs(cvmGet(matDiff_full, i, 0)) < RANSAC_TOL_STRICT)
					filtered_index.push_back(i);
			}
		}
	}

	if ((int)filtered_index.size() >= 3 && (int)filtered_index.size() > (int)(RANSAC_LEAST_SIZE * max_counter))
	{
		matA_filtered = cvCreateMat((int)filtered_index.size(), 3, CV_32FC1);
		matB_filtered = cvCreateMat((int)filtered_index.size(), 1, CV_32FC1);

		y_min = y_max = pt[filtered_index[0]].y;
		for (int i = 0; i < (int)filtered_index.size(); i++)
		{
			y_cur = pt[filtered_index[i]].y;
			if (y_cur > y_max)
				y_max = y_cur;
			if (y_cur < y_min)
				y_min = y_cur;

			cvmSet(matA_filtered, i, 0, cvmGet(matA_full, filtered_index[i], 0));
			cvmSet(matA_filtered, i, 1, cvmGet(matA_full, filtered_index[i], 1));
			cvmSet(matA_filtered, i, 2, cvmGet(matA_full, filtered_index[i], 2));
			cvmSet(matB_filtered, i, 0, cvmGet(matB_full, filtered_index[i], 0));
		}
		cvSolve(matA_filtered, matB_filtered, matX_opt, CV_SVD);

		cvReleaseMat(&matA_filtered);
		cvReleaseMat(&matB_filtered);

		a = (float)cvmGet(matX_opt, 0, 0);
		b = (float)cvmGet(matX_opt, 1, 0);
		c = (float)cvmGet(matX_opt, 2, 0);
		belief = (double)filtered_index.size() / max_counter;

		//if (fabs(a) <= 0.05 && fabs(b) <= 0.5 && (y_max - y_min) > 1.0)//tan 20131126 10)
		if (1)//tian 20140809
			param = cvScalar(a, b, c, belief);
		else
		{
			param = cvScalar(0);

			//cout <<" massege 1 " << " a = " << a << " b = " << b  <<  " c = " << c <<" y_max = " << y_max<< "  y_min=" << y_min << endl;
		}
	}
	else
	{
		param = cvScalar(0);
		//cout << " message 2 " <<  "size =" << filtered_index.size() << " least " << RANSAC_LEAST_SIZE * max_counter << endl;
	}

	cvReleaseMat(&matA);
	cvReleaseMat(&matB);
	cvReleaseMat(&matX);
	cvReleaseMat(&matX_opt);
	cvReleaseMat(&matA_full);
	cvReleaseMat(&matB_full);
	cvReleaseMat(&matX_full);
	cvReleaseMat(&matDiff_full);

	return param;
}

CvScalar planeRansac(const vector<CvPoint3D32f>& pt) {

	CvMat *matA = NULL, *matB = NULL, *matX = NULL, *matX_opt = NULL;
	CvMat *matA_full = NULL, *matB_full = NULL, *matX_full = NULL, *matDiff_full = NULL;
	CvMat *matA_filtered = NULL, *matB_filtered = NULL;

	const int RANSAC_LOOP = 50;
	const float RANSAC_TOL = 0.6;
	const float RANSAC_TOL_STRICT = 0.4;
	const float RANSAC_LEAST_SIZE = 0.5;

	int counter = 0, max_counter = 0;
	int s[3];
	double x, y, z, a, b, c, y_min, y_max, y_cur;
	double belief;
	vector<int> filtered_index;
	CvScalar param = cvScalar(0);

	if ((int)pt.size() <= 3) //if ((int)pt.size() < 10)          // tan 20131126 if ((int)pt.size() < 10)
		return cvScalar(0);

	matA = cvCreateMat(3, 3, CV_32FC1);
	matB = cvCreateMat(3, 1, CV_32FC1);
	matX = cvCreateMat(3, 1, CV_32FC1);
	matX_opt = cvCreateMat(3, 1, CV_32FC1);
	matA_full = cvCreateMat((int)pt.size(), 3, CV_32FC1);
	matB_full = cvCreateMat((int)pt.size(), 1, CV_32FC1);
	matX_full = cvCreateMat((int)pt.size(), 1, CV_32FC1);
	matDiff_full = cvCreateMat((int)pt.size(), 1, CV_32FC1);

	for (int i = 0; i < (int)pt.size(); i++)
	{
		x = (double)pt[i].x / 1000;
		y = (double)pt[i].z / 1000;
		z = (double)pt[i].y / 1000;
		cvmSet(matA_full, i, 0, x);
		cvmSet(matA_full, i, 1, z);
		cvmSet(matA_full, i, 2, 1);
		cvmSet(matB_full, i, 0, y);
	}

	//cvSolve(matA_full, matB_full, matX_opt, CV_SVD);

	//a = cvmGet(matX_opt, 0, 0);
	//b = cvmGet(matX_opt, 1, 0);
	//c = cvmGet(matX_opt, 2, 0);

	//param = cvScalar(a, b, c, 1);

	for (int cycle_counter = 0; cycle_counter < RANSAC_LOOP; cycle_counter++)
	{
		s[0] = (int)((double)rand() / RAND_MAX * ((int)pt.size() - 1));
		do
		{
			s[1] = (int)((double)rand() / RAND_MAX * ((int)pt.size() - 1));
		} while (s[0] == s[1]);
		do
		{
			s[2] = (int)((double)rand() / RAND_MAX * ((int)pt.size() - 1));
		} while (s[2] == s[0] || s[2] == s[1]);

		for (int i = 0; i < 3; i++)
		{
			x = (double)pt[s[i]].x / 1000;
			y = (double)pt[s[i]].z / 1000;
			z = (double)pt[s[i]].y / 1000;
			cvmSet(matA, i, 0, x);
			cvmSet(matA, i, 1, z);
			cvmSet(matA, i, 2, 1);
			cvmSet(matB, i, 0, y);
		}

		cvSolve(matA, matB, matX);
		cvGEMM(matA_full, matX, 1, matB_full, -1, matDiff_full);

		counter = 0;
		for (int i = 0; i < (int)pt.size(); i++)
		{
			if (fabs(cvmGet(matDiff_full, i, 0)) < RANSAC_TOL)
				counter++;
		}

		if (counter > max_counter)
		{
			cvCopy(matX, matX_opt);
			max_counter = counter;
			filtered_index.clear();
			for (int i = 0; i < (int)pt.size(); i++)
			{
				if (fabs(cvmGet(matDiff_full, i, 0)) < RANSAC_TOL_STRICT)
					filtered_index.push_back(i);
			}
		}
	}

	if ((int)filtered_index.size() >= 3 && (int)filtered_index.size() > (int)(RANSAC_LEAST_SIZE * max_counter))
	{
		matA_filtered = cvCreateMat((int)filtered_index.size(), 3, CV_32FC1);
		matB_filtered = cvCreateMat((int)filtered_index.size(), 1, CV_32FC1);

		y_min = y_max = pt[filtered_index[0]].y;
		for (int i = 0; i < (int)filtered_index.size(); i++)
		{
			y_cur = pt[filtered_index[i]].y;
			if (y_cur > y_max)
				y_max = y_cur;
			if (y_cur < y_min)
				y_min = y_cur;

			cvmSet(matA_filtered, i, 0, cvmGet(matA_full, filtered_index[i], 0));
			cvmSet(matA_filtered, i, 1, cvmGet(matA_full, filtered_index[i], 1));
			cvmSet(matA_filtered, i, 2, cvmGet(matA_full, filtered_index[i], 2));
			cvmSet(matB_filtered, i, 0, cvmGet(matB_full, filtered_index[i], 0));
		}
		cvSolve(matA_filtered, matB_filtered, matX_opt, CV_SVD);

		cvReleaseMat(&matA_filtered);
		cvReleaseMat(&matB_filtered);

		a = (double)cvmGet(matX_opt, 0, 0);
		b = (double)cvmGet(matX_opt, 1, 0);
		c = (double)cvmGet(matX_opt, 2, 0);
		belief = (double)filtered_index.size() / max_counter;

		//if (fabs(a) <= 0.05 && fabs(b) <= 0.5 && (y_max - y_min) > 1.0)//tan 20131126 10)
		if (1)//tian 20140809
			param = cvScalar(a, b, c, belief);
		else
		{
			param = cvScalar(0);

			//cout <<" massege 1 " << " a = " << a << " b = " << b  <<  " c = " << c <<" y_max = " << y_max<< "  y_min=" << y_min << endl;
		}
	}
	else
	{
		param = cvScalar(0);
		//cout << " message 2 " <<  "size =" << filtered_index.size() << " least " << RANSAC_LEAST_SIZE * max_counter << endl;
	}

	cvReleaseMat(&matA);
	cvReleaseMat(&matB);
	cvReleaseMat(&matX);
	cvReleaseMat(&matX_opt);
	cvReleaseMat(&matA_full);
	cvReleaseMat(&matB_full);
	cvReleaseMat(&matX_full);
	cvReleaseMat(&matDiff_full);

	return param;
}
	
bool ConcaveDetector::IdealizeFrame() {
	/// 1.Traverse All Grids Inbox, Collect One Point For Each Grid
	vector<CvPoint3D32f> pointsPool;
	for (int n = 0; n < 10; ++n){
		// ten boxs
		int leftIndex = floor(_leftEdgeArray[n].x/200+250);
		int rightIndex = floor(_rightEdgeArray[n].x/200 + 250);
		int buttomIndex = QT_BOX_LENGTH * n / 200+250;
		int topIndex = QT_BOX_LENGTH * (n + 1) / 200+250;
		for (int i = buttomIndex; i < topIndex; ++i){
			for (int j = leftIndex; j < rightIndex; ++j){
				if (_localGrids[i][j]._dotCount >= 2)
				{
					pointsPool.push_back(cvPoint3D32f((j-250)*200, (i-250)*200, _localGrids[i][j]._minHeight));
				}
			}
		}
	}
	/// cout << "1.Collect Points for Fitting Success!" << endl;;
	/// 2.Plane-fitting
	CvScalar params = planeRansac(pointsPool);
	/// cout << "2.Plane fitting Success!" << endl;
	/// 3.Get Transform Mat
	/// cout << "A = " << params.val[0] << " B = " << params.val[1] << " C = " << params.val[2] << " belief = " << params.val[3] << endl;
	double ratioA = -params.val[0] / params.val[2];
	double ratioB = -params.val[1] / params.val[2];
	double ratioC = 1 / params.val[2];
	double Sigma = pow(ratioA, 2) + pow(ratioB, 2) + pow(ratioC, 2);
	double Vy1 = -ratioA*ratioB / Sigma;
	double Vy2 = -pow(ratioB, 2) / Sigma + 1;
	double Vy3 = -ratioB*ratioC / Sigma;
	double Vz1 = -ratioA;
	double Vz2 = -ratioB;
	double Vz3 = 1;
	double Vx1 = Vy2*Vz3 - Vy3*Vz2;
	double Vx2 = Vy1*Vz2 - Vy1*Vz3;
	double Vx3 = Vy1*Vz2 - Vy2*Vz1;

	/// Mat T = (Mat_<double>(3, 3) << Vx1, Vx2, Vx3, Vy1, Vy2, Vy3, Vz1, Vz2, Vz3);
	/// cout << "3.Get Transform Mat Success!" << endl;
	/// 4.Calibrate All Coordinate
	for (int i = 0; i < HDL_LASER_NUMBER_ROAD; ++i){
		int count = m_point[i].pt_count;
		for (int j = 0; j < count; ++j){
			double x = m_point[i].pt[j].x;
			double y = m_point[i].pt[j].y;
			double z = m_point[i].pt[j].z;
			m_point[i].pt[j].x = Vx1*x + Vx2*y + Vx3*z;
			m_point[i].pt[j].y = Vy1*x + Vy2*y + Vy3*z;
			m_point[i].pt[j].z = Vz1*x + Vz2*y + Vz3*z;
		}
	}
	/*for (int i = 0; i < HDL_LASER_NUMBER_ROAD; ++i){
		int count = m_point[i].pt_count;
		for (int j = 0; j < count; ++j){
			Mat_<double> temp;
			Mat input = (Mat_<double>(3, 1) << m_point[i].pt[j].x, m_point[i].pt[j].y, m_point[i].pt[j].z);
			temp = T*input;
			m_point[i].pt[j].x = temp(0, 0);
			m_point[i].pt[j].y = temp(1, 0);
			m_point[i].pt[j].z = temp(2, 0);
		}
	}*/
	/// cout << "4.Calibrate All Coordinate Success!" << endl;
	/// 5.Clear Local Coordinate System
	initLocalGrids();
	/// cout << "5.Clear Local Coordinate System!" << endl;
	return true;
}

bool ConcaveDetector::readVehicleData(const char *filepath, Vehicle value){
	//Vehicle *vehicledata = new Vehicle();
	char directory[200];
	sprintf(directory, "%s.vehicle", filepath);
	_inVehicleStream.open(directory);
	if (!_inVehicleStream)
	{
		cout << "NegativeObstacleDetector can not open this vehicle file:%s" << filepath << endl;
		return false;
	}
	else 
	{
		if (!_inVehicleStream.eof())
		{
			_inVehicleStream >> value.x >> value.y >> value.theta;
			//value.x = vehicledata->x;
			return true;
		}
		else
		{
			cout << "NegativeObstacleDetector read vehicle data done!" << endl;
			return false;
		}
	}
}

bool ConcaveDetector::locateTargetRegionEdge(){
	/// clear edage
	for (int i = 0; i < 10; i++){
		_leftEdgeArray[0].x  = 0;
		_rightEdgeArray[0].x = 0;
	}

	int ul = 250; ///<  0 ~  250   unit is 200mm
	int ur = 250; ///<  250 ~ +500   unit is 200mm
	int section = 0;
	for (int v = 250; v < 500; v+=25) {  //
		float tmpl = 0.5;
		for (int i = 0; i < 25; i++){
			tmpl = MAX( _localGrids[v+i][ul]._pForObstacle, tmpl);
			if (abs(tmpl - 0.8) < 0.001)
			{
				break;
			}
		}
		
		while (tmpl < 0.8){
			ul -= 1;
			if (ul<200)
			{
				ul = 200;
				break;
			}
			for (int i = 0; i < 25; i++){
				tmpl = MAX(_localGrids[v + i][ul]._pForObstacle, tmpl);
				if (abs(tmpl - 0.8) < 0.001)
				{
					break;
				}
			}
		}
		float tmpr = 0.5;
		for (int i = 0; i < 25; i++){
			tmpr = MAX(_localGrids[v+i][ur]._pForObstacle, tmpr);
			if (abs(tmpr - 0.8) < 0.001)
			{
				break;
			}
		}
		while (tmpr < 0.8){
			ur += 1;
			if (ur > 300)
			{
				ur = 300;
				break;
			}
			for (int i = 0; i < 25; i++){
				tmpr = MAX(_localGrids[v+i][ur]._pForObstacle, tmpr);
				if (abs(tmpr - 0.8) < 0.001)
				{
					break;
				}
			}
		}
		// unit is mm
		_leftEdgeArray[section]  = Point2f((float)(ul-250)*200, (float)section*5000);
		_rightEdgeArray[section] = Point2f((float)(ur-250)*200, (float)section*5000);
		section++;
		int mid = (ul + ur) / 2;
		ul = mid; ur = mid;
	}
	return true;
}

bool ConcaveDetector::collectPointInbox(){
	// Init the m_point_inbox
	for (int i = 0; i < HDL_LASER_NUMBER_ROAD; i++){
		m_point_inbox[i].pt_count = 0;
	}
	// Fill the m_point_inbox
	for (int i = 0; i < HDL_LASER_NUMBER_ROAD; i++){
		int count = m_point[i].pt_count;
		for (int j = 0; j < count; j++){
			for (int k = 0; k < 10; k++){
				if (m_point[i].pt[j].x >= _leftEdgeArray[k].x  && \
					m_point[i].pt[j].x < _rightEdgeArray[k].x  && \
					m_point[i].pt[j].y >= 5000 * k             && \
					m_point[i].pt[j].y < 5000 * (k + 1))
				{
					m_point[i].road_type[j] = 1;
					m_point_inbox[i].pt[m_point_inbox[i].pt_count] = m_point[i].pt[j];
					m_point_inbox[i].road_type[m_point_inbox[i].pt_count] = 1;
					m_point_inbox[i].pt_count++;
					break;
				}
			}
		}
	}
	return true;
}

bool ConcaveDetector::locateFeaturePointUsingLinerRansac(){
	for (int i = 0; i < HDL_LASER_NUMBER_ROAD; i++)
	{
		Point_laser_t aLaser = m_point_inbox[i];
		vector<CvPoint3D32f> pts;
		double sigma_rou = 0;		///< sum rou
		double average_rou = 0;		///< average rou
		double tol = 0;				///< thresh value
		for (int j = 0; j < aLaser.pt_count; j++){
			//pts.push_back(aLaser.pt[j]);
			CvPoint3D32f tmp;
			double rou = sqrt((float)aLaser.pt[j].x*(float)aLaser.pt[j].x + (float)aLaser.pt[j].y*(float)aLaser.pt[j].y);
			tmp.x = 1 / rou;
			tmp.y = (double)aLaser.pt[j].x;
			tmp.z = (double)aLaser.pt[j].z;
			pts.push_back(tmp);

			// get the average rou
			sigma_rou += rou;
		}
		average_rou = sigma_rou / aLaser.pt_count;		 /// unit is mm
		tol = (1 / average_rou - 1 / (average_rou + obj_height_thr)); /// unit is mm

		CvScalar prams = lineRansac(pts, tol * 1000); // prams used specified for unit meter
		if (!(prams.val[0] + prams.val[1]))
		{
			cout << "\nprams are zero, and the row is " << i << endl;
		}

		// search for feature point 
		int dot_number = m_point_inbox[i].pt_count;
		for (int j = 0; j < dot_number; j++){
			double x = m_point_inbox[i].pt[j].x;
			double y = m_point_inbox[i].pt[j].y;
			double z = m_point_inbox[i].pt[j].z;

			double rou = sqrt(x*x + y*y);

			double distance = ((prams.val[0] * x * 1E-6 + prams.val[1] * 1E-3) - 1 / rou); // unit is mm
			if (distance >= tol && prams.val[3] > 0.6)
			{
				int grid_row = (int)floor(y / 200) + 250;
				int grid_col = (int)floor(x / 200) + 250;
				// update grid's probality in local coordinate system
				m_point_inbox[i].road_type[j] = 2;// obstacle point
				_localGrids[grid_row][grid_col]._pForNegativeObstacle = 0.8;
			}
		}
		int count = 0;
		for (int j = 0; j < dot_number; j++){
			
			if (m_point_inbox[i].road_type[j] == 2)
			{
				count++;
			}
		}
		if (count > 100)
		{
			cout << "Feature Points Number in Row " << i << " is over 100" << endl;
		}
	}
	return true;
}

bool ConcaveDetector::locateFeaturePointUsingPolyRansac(){
	for (int i = 0; i < HDL_LASER_NUMBER_ROAD; i++)
	{
		Point_laser_t aLaser = m_point_inbox[i];
		vector<CvPoint3D32f> pts;
		for (int j = 0; j < aLaser.pt_count; j++){
			//pts.push_back(aLaser.pt[j]);
			CvPoint3D32f tmp;
			tmp.x = (float)aLaser.pt[j].y;
			tmp.y = (float)aLaser.pt[j].x;
			tmp.z = (float)aLaser.pt[j].z;
			pts.push_back(tmp);
		}

		CvScalar prams = polyRansac(pts); // prams used specified for unit meter

		// search for feature point 
		int dot_number = m_point_inbox[i].pt_count;
		for (int j = 0; j < dot_number; j++){
			int x = m_point_inbox[i].pt[j].x;
			int y = m_point_inbox[i].pt[j].y;
			int z = m_point_inbox[i].pt[j].z;

			double distance = (y - (prams.val[0] / 1000 * x * x + prams.val[1] * x + 1000 * prams.val[2]));
			if (distance >= 600 && prams.val[3] > 0.6)
			{
				int grid_row = (int)floor(y / 200) + 250;
				int grid_col = (int)floor(x / 200) + 250;
				// update grid's probality in local coordinate system
				m_point[i].road_type[j] = 2;// obstacle point
				_localGrids[grid_row][grid_col]._pForNegativeObstacle = 0.8;
			}
		}
	}
	return true;
}

bool ConcaveDetector::runDetector(RunMode runmode, PlaneFittingMode planeMode, FittingMode fitting_selector) {
	/// 1.Read
	if (runmode == ONLINE)
		if (!ReadDataFromShm()) return false;
		//else;
	else 
		if (!ReadDataFromFile())	 return false;
		//else;
	/// 2.Transform Array
	if (!ProcessArray())	     return false;
	/// 3.
	if (!updateLocalGridsUsingRelativeHeight())	return false;
	if (!locateTargetRegionEdge())				return false;
	/// 4.Judge Plane Fitting Is On
	if (planeMode == PLANE_ON)
	{
		if (!IdealizeFrame())						return false;
		if (!updateLocalGridsUsingAbsoluteHeight())	return false;
		if (!locateTargetRegionEdge())				return false;
	}
	if (!collectPointInbox())						return false;
	/// 4.
	if (fitting_selector == LINE_FITTING)
		if (!locateFeaturePointUsingLinerRansac())	return false;
	if (fitting_selector == POLY_FITTING)
		if (!locateFeaturePointUsingPolyRansac())   return false;
//////////////////////////////////////////////////////////////////////////
	/// 5.Update the freezed coordinate system
	if (!updateGlobalGrid())						return false;
	return true;
}

bool ygr_process_array(vector<LPoint> hdl_pts, Point_laser_t *m_point_array){
	//m_point_array = new Point_laser_t[HDL_LASER_NUMBER];
	// 1.store origin points into two dimentional array
	int c, count[HDL_LASER_NUMBER];
	for (int i = 0; i < HDL_LASER_NUMBER; i++){
		count[i] = 0;
	}
	int point_count = hdl_pts.size();
	for (int i = 0; i < point_count; i++){
		c = hdl_pts[i].c;
		m_point_array[c].pt[count[c]].x = hdl_pts[i].x;
		m_point_array[c].pt[count[c]].y = hdl_pts[i].y;
		m_point_array[c].pt[count[c]].z = hdl_pts[i].z;
		m_point_array[c].road_type[count[c]] = 0;
		m_point_array[c].pt[count[c]].dist = hdl_pts[i].dist;
		m_point_array[c].pt[count[c]].i = hdl_pts[i].i;
		m_point_array[c].pt[count[c]].c = hdl_pts[i].c;
		m_point_array[c].pt[count[c]].rot = 9000 - hdl_pts[i].rot - int(laser_angle[c] * 100);

		if (m_point_array[c].pt[count[c]].rot < 0)
		{
			m_point_array[c].pt[count[c]].rot += 36000;
		}

		//m_point[c].pt[count[c]].rot = m_cloud[i].rot;
		//m_point[c].is_road[count[c]] = false;
		//m_point[c].pt[count[c]].index = i ;
		count[c]++;
	}
	// 2.update the number of points in every line
	for (int i = 0; i < HDL_LASER_NUMBER; i++)
	{
		m_point_array[i].pt_count = count[i];
	}

	// 3.sort the 64 lasers from near to far
	int temp[] = { 39, 40, 43, 44, 33, 34, 37, 38, 41, 42,
		47, 48, 51, 52, 55, 56, 45, 46, 49, 50,
		53, 54, 59, 60, 63, 64, 35, 36, 57, 58,
		61, 62, 7, 8, 11, 12, 1, 2, 5, 6,
		9, 10, 15, 16, 19, 20, 23, 24, 13, 14,
		17, 18, 21, 22, 27, 28, 31, 32, 3, 4,
		25, 26, 29, 30
	};
	int m_laser_index[HDL_LASER_NUMBER];
	for (int i = 0; i < HDL_LASER_NUMBER; i++){
		m_laser_index[i] = temp[i];
	}

	Point_laser* pl = new Point_laser[HDL_LASER_NUMBER];
	for (int i = 0; i < HDL_LASER_NUMBER; i++)
	{
		pl[i] = m_point_array[m_laser_index[i] - 1];
	}
	for (int i = 0; i < HDL_LASER_NUMBER; i++)
	{
		m_point_array[i] = pl[i];
	}
	delete[] pl;
	return true;
}

bool ygr_locate_feature_point_using_liner_ransac(Point_laser_t *m_point_array, vector<LPoint> hdl_pts_out)
{
	for (int i = 0; i < HDL_LASER_NUMBER_ROAD; i++)
	{
		Point_laser_t aLaser = m_point_array[i];
		vector<CvPoint3D32f> pts;
		double sigma_rou = 0;		///< sum rou
		double average_rou = 0;		///< average rou
		double tol = 0;				///< thresh value
		for (int j = 0; j < aLaser.pt_count; j++){
			//pts.push_back(aLaser.pt[j]);
			CvPoint3D32f tmp;
			double rou = sqrt((float)aLaser.pt[j].x*(float)aLaser.pt[j].x + (float)aLaser.pt[j].y*(float)aLaser.pt[j].y);
			tmp.x = 1 / rou;
			tmp.y = (double)aLaser.pt[j].x;
			tmp.z = (double)aLaser.pt[j].z;
			pts.push_back(tmp);

			// get the average rou
			sigma_rou += rou;
		}
		average_rou = sigma_rou / aLaser.pt_count;		 /// unit is mm
		tol = (1 / average_rou - 1 / (average_rou + obj_height_thr)); /// unit is mm

		CvScalar prams = lineRansac(pts, tol * 1000); // prams used specified for unit meter
		if (!(prams.val[0] + prams.val[1]))
		{
			cout << "\nprams are zero, and the row is " << i << endl;
		}

		// search for feature point 
		int dot_number = m_point_array[i].pt_count;
		for (int j = 0; j < dot_number; j++){
			double x = m_point_array[i].pt[j].x;
			double y = m_point_array[i].pt[j].y;
			double z = m_point_array[i].pt[j].z;

			double rou = sqrt(x*x + y*y);

			double distance = ((prams.val[0] * x * 1E-6 + prams.val[1] * 1E-3) - 1 / rou); // unit is mm
			if (distance >= tol && prams.val[3] > 0.6)
			{
				//int grid_row = (int)floor(y / 200) + 250;
				//int grid_col = (int)floor(x / 200) + 250;
				// update grid's probality in local coordinate system
				//m_point_array[i].road_type[j] = 2;// obstacle point
				//_localGrids[grid_row][grid_col]._pForNegativeObstacle = 0.8;
				hdl_pts_out.push_back(m_point_array[i].pt[j]);
			}
		}
	}
	return true;
}

bool ygr_locate_feature_point_using_poly_ransac(Point_laser_t *m_point_array, vector<LPoint> hdl_pts_out){
	for (int i = 0; i < HDL_LASER_NUMBER_ROAD; i++)
	{
		Point_laser_t aLaser = m_point_array[i];
		vector<CvPoint3D32f> pts;
		for (int j = 0; j < aLaser.pt_count; j++){
			//pts.push_back(aLaser.pt[j]);
			CvPoint3D32f tmp;
			tmp.x = (float)aLaser.pt[j].y;
			tmp.y = (float)aLaser.pt[j].x;
			tmp.z = (float)aLaser.pt[j].z;
			pts.push_back(tmp);
		}

		CvScalar prams = polyRansac(pts); // prams used specified for unit meter

		// search for feature point 
		int dot_number = m_point_array[i].pt_count;
		for (int j = 0; j < dot_number; j++){
			int x = m_point_array[i].pt[j].x;
			int y = m_point_array[i].pt[j].y;
			int z = m_point_array[i].pt[j].z;

			double distance = (y - (prams.val[0] / 1000 * x * x + prams.val[1] * x + 1000 * prams.val[2]));
			if (distance >= 600 && prams.val[3] > 0.6)
			{
				/*int grid_row = (int)floor(y / 200) + 250;
				int grid_col = (int)floor(x / 200) + 250;
				// update grid's probality in local coordinate system
				m_point[i].road_type[j] = 2;// obstacle point
				_localGrids[grid_row][grid_col]._pForNegativeObstacle = 0.8;*/
				hdl_pts_out.push_back(m_point_array[i].pt[j]);
			}
		}
	}
	return true;
}

bool ygr_run_detector(vector<LPoint> hdl_pts, vector<LPoint> hdl_pts_out, FittingMode fitting_selector){
	Point_laser_t *m_point_array;
	vector<LPoint> hdl_pts_out;
	if (!ygr_process_array(hdl_pts, m_point_array)) return false;
	if (fitting_selector == LINE_FITTING)
		if (!ygr_locate_feature_point_using_liner_ransac(m_point_array, hdl_pts_out))	return false;
	if (fitting_selector == POLY_FITTING)
		if (!ygr_locate_feature_point_using_poly_ransac(m_point_array, hdl_pts_out))   return false;
}

bool ConcaveDetector::ygrRun(const vector<HDLPoint> &hdl_pts, vector<HDLPoint>& hdl_pts_out, PlaneFittingMode planeMode, FittingMode fitting_selector) {
	m_cloud_count = hdl_pts.size();
	for (int i = 0; i < m_cloud_count; ++i){
		m_cloud[i].x = (int)hdl_pts[i].x * 1000;
		m_cloud[i].y = (int)hdl_pts[i].y * 1000;
		m_cloud[i].z = (int)hdl_pts[i].z * 1000;
		m_cloud[i].dist = (int)hdl_pts[i].dist * 1000;
		m_cloud[i].i = hdl_pts[i].i;
		m_cloud[i].c = hdl_pts[i].c;
		m_cloud[i].rot = hdl_pts[i].rot;
	}

	/// 2.Transform Array
	if (!ProcessArray())	     return false;
	/// 3.
	if (!updateLocalGridsUsingRelativeHeight())	return false;
	if (!locateTargetRegionEdge())				return false;
	/// 4.Judge Plane Fitting Is On
	if (planeMode == PLANE_ON)
	{
		if (!IdealizeFrame())						return false;
		if (!updateLocalGridsUsingAbsoluteHeight())	return false;
		if (!locateTargetRegionEdge())				return false;
	}
	if (!collectPointInbox())						return false;
	/// 4.
	if (fitting_selector == LINE_FITTING)
		if (!locateFeaturePointUsingLinerRansac())	return false;
	if (fitting_selector == POLY_FITTING)
		if (!locateFeaturePointUsingPolyRansac())   return false;
	//////////////////////////////////////////////////////////////////////////
	/// 5.Update the freezed coordinate system
	//if (!updateGlobalGrid())						return false;

	for (int i = 0; i < HDL_LASER_NUMBER_ROAD; i++)
	{
		// search for feature point 
		int dot_number = m_point_inbox[i].pt_count;
		for (int j = 0; j < dot_number; j++){
			if (m_point[i].road_type[j] == 2)
			{
				HDLPoint *temp = new HDLPoint();
				temp->x = m_point_inbox[i].pt[j].x / 1000;
				temp->y = m_point_inbox[i].pt[j].y / 1000;
				temp->z = m_point_inbox[i].pt[j].z / 1000;
				temp->dist = m_point_inbox[i].pt[j].dist / 1000;
				hdl_pts_out.push_back(*temp);
			}
		}
	}
	return true;
}
