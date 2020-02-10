#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

class PoseEstimation
{
public:
	PoseEstimation();

	bool readIntrinsicParameters(string fileName);

public:
	//
	// Intrinsic Camera Parameters
	//
	bool	_intrinsic;
	Mat		_cameraIntrinsic;
	Mat		_cameraDistortion;

};

