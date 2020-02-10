#include "PoseEstimation.h"

#define atd at<double>

PoseEstimation::PoseEstimation()
{
	_cameraIntrinsic = Mat::zeros(3, 3, DataType<double>::type);
	_cameraDistortion = Mat::zeros(4, 1, DataType<double>::type);
}

bool PoseEstimation::readIntrinsicParameters(string fileName)
{
	fstream file(fileName.c_str(), fstream::in | fstream::out);

	if (!file.is_open()) 
		return false;

	_cameraIntrinsic.atd(2, 2) = 1;

	file >> _cameraIntrinsic.atd(0, 0);
	file >> _cameraIntrinsic.atd(1, 1);
	file >> _cameraIntrinsic.atd(0, 2);
	file >> _cameraIntrinsic.atd(1, 2);

	for (int i = 0; i < 4; ++i)
		file >> _cameraDistortion.atd(i);
	file.close();

	_intrinsic = true;

	return _intrinsic;
}