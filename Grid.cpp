#include "Grid.h"

Grid::Grid() {
	_x = 0;
	_y = 0;
	_dotCount = 0;					//laser dot number in this grid
	_maxHeight = -10000;
	_minHeight = +10000;
	_pForObstacle = 0.5;
	_pForNegativeObstacle = 0.5;
}
Grid::~Grid() {

}

int Grid::getXValue(){
	return _x;
}
int Grid::getYValue(){
	return _y;
}
int Grid::getDotCount(){
	return _dotCount;
}

int Grid::getMaxHeight(){
	return _maxHeight;
}
int Grid::getMinHeight(){
	return _minHeight;
}
double Grid::getPForObstacle(){
	return _pForObstacle;
}
double Grid::getPForNegativeObstacle(){
	return _pForNegativeObstacle;
}

void Grid::setXValue(int newXValue){
	_x = newXValue;
}
void Grid::setYValue(int newYValue){
	_y = newYValue;
}
void Grid::setDotCount(int newDotCount){
	_dotCount = newDotCount;
}
void Grid::setMaxHeight(int newMaxHeight){
	_maxHeight = newMaxHeight;
}
void Grid::setMinHeight(int newMinHeight){
	_minHeight = newMinHeight;
}
void Grid::setPForObstacle(double newPForObstacle){
	_pForObstacle = newPForObstacle;
}
void Grid::setPForNegativeObstacle(double newPForNegativeObstacle){
	_pForNegativeObstacle = newPForNegativeObstacle;
}