#include "FingerTip.h"


FingerTip::FingerTip()
{
}

FingerTip::FingerTip(int index, Point2f point, float dist)
{
	_index = index;
	_point = point;
	_dist = dist;
}

FingerTip::~FingerTip()
{
}

bool FingerTip::operator < (const FingerTip &f)
{
	return _point.x < f._point.x;
}
