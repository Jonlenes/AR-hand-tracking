#include "Quaternion.h"


Quaternion::Quaternion()
{
}


Quaternion::~Quaternion()
{
}

void Quaternion::matrix2Quaternion(Mat &m, Mat &q)
{
	//cout << m << endl; DEBUG

	float w = float(sqrt(1.0 + m.at<double>(0, 0) + m.at<double>(1, 1) + m.at<double>(2, 2)) / 2.0);
	float w4 = float(4.0 * w);
	float x = float(m.at<double>(2, 1) - m.at<double>(1, 2)) / w4;
	float y = float(m.at<double>(0, 2) - m.at<double>(2, 0)) / w4;
	float z = float(m.at<double>(1, 0) - m.at<double>(0, 1)) / w4;

	q.at<double>(0) = w;
	q.at<double>(1) = x;
	q.at<double>(2) = y;
	q.at<double>(3) = z;
}

/* ref: http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm */
void Quaternion::quaternion2Matrix(Mat &q, Mat &m)
{
	float sqw = float(q.at<double>(0) * q.at<double>(0));
	float sqx = float(q.at<double>(1) * q.at<double>(1));
	float sqy = float(q.at<double>(2) * q.at<double>(2));
	float sqz = float(q.at<double>(3) * q.at<double>(3));

	m.at<double>(0, 0) = sqx - sqy - sqz + sqw;	// since sqw + sqx + sqy + sqz =1
	m.at<double>(1, 1) = -sqx + sqy - sqz + sqw;
	m.at<double>(2, 2) = -sqx - sqy + sqz + sqw;

	float tmp1 = float(q.at<double>(1) * q.at<double>(2));
	float tmp2 = float(q.at<double>(3) * q.at<double>(0));

	m.at<double>(1, 0) = float(2.0 * (tmp1 + tmp2));
	m.at<double>(0, 1) = float(2.0 * (tmp1 - tmp2));

	tmp1 = float(q.at<double>(1) * q.at<double>(3));
	tmp2 = float(q.at<double>(2) * q.at<double>(0));

	m.at<double>(2, 0) = float(2.0 * (tmp1 - tmp2));
	m.at<double>(0, 2) = float(2.0 * (tmp1 + tmp2));

	tmp1 = float(q.at<double>(2) * q.at<double>(3));
	tmp2 = float(q.at<double>(1) * q.at<double>(0));

	m.at<double>(2, 1) = float(2.0 * (tmp1 + tmp2));
	m.at<double>(1, 2) = float(2.0 * (tmp1 - tmp2));
}

