/*!
* \file
* \brief
* 
* detail
*/

/// \brief Grid Object
/// 
/// Used for present a single grid
class Grid {
public:
	double _pForObstacle;
	double _pForNegativeObstacle;
	int _dotCount;					///< laser dot number in this grid
	int _maxHeight;					///< unit is mm
	int _minHeight;					///< unit is mm

private:
	int _x;							///< unit is mm
	int _y;							///< unit is mm

public:
	Grid();
	~Grid();

	(int)getXValue();
	(int)getYValue();
	(int)getDotCount();

	(int)getMaxHeight();
	(int)getMinHeight();
	(double)getPForObstacle();
	(double)getPForNegativeObstacle();

	(void)setXValue(int newXValue);
	(void)setYValue(int newYValue);
	(void)setDotCount(int newDotCount);
	(void)setMaxHeight(int newMaxHeight);
	(void)setMinHeight(int newMinHeight);
	(void)setPForObstacle(double newPForObstacle);
	(void)setPForNegativeObstacle(double newPForNegativeObstacle);
};