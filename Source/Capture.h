#pragma once

#include <opencv2\opencv.hpp>

using namespace cv;

#define CAPTURE_SIZE_WIDTH  640
#define CAPTURE_SIZE_HEIGHT 480

class Capture
{
public:
	Capture();
	~Capture();

	bool initialize(int index);
	void terminate();

	Mat &nextFrame();

	int getFrameWidth() const;
	int getFrameHeigth() const;

private:
	VideoCapture cap;
	Mat frame;
};

