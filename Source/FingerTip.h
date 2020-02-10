#pragma once

#include <opencv2\opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

class FingerTip
{
public:
	FingerTip(int index, Point2f point, float dist);
	FingerTip();
	~FingerTip();

	bool operator < (const FingerTip &f);

	int			_index;
	Point2f		_point;
	float		_dist;
};

