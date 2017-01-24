#include "Header.h"
#include "ConcaveDetector.h"

int main(int argc, char** argv)
{	
	//Register();
	ConcaveDetector *m_concave = new ConcaveDetector();
	if (!m_concave->Initialize("../../Data/20140713_155407_holes.hdl"))
	{
		return false;
	}
	m_concave->RunConcaveDetector();
	delete m_concave;
	getchar();
	return 0;
}