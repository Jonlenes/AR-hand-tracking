#pragma once

#include <opencv2\opencv.hpp>

#include "ArException.h"

using namespace cv;
using namespace std;

class HandRegion
{
public:
	HandRegion();

	Mat getHandRegion(Mat &srcImage);
};

