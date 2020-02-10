#pragma once

#include <vector>
#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

/*
Em matemática , os quaternões são um sistema numérico que amplia os números complexos.
Eles foram descritos pela primeira vez pelo matemático irlandês William Rowan Hamilton 
em 1843 [1] [2] e aplicados à mecânica no espaço tridimensional . Uma característica 
dos quaternões é que a multiplicação de dois quaternões não é comutativa. Hamilton 
definiu um quaternion como o quociente de duas linhas direcionadas em um espaço 
tridimensional [3] ou equivalentemente como o quociente de dois vetores .
*/


class Quaternion
{
public:
	Quaternion();
	~Quaternion();

	static void matrix2Quaternion(Mat &m, Mat &q);
	static void quaternion2Matrix(Mat &q, Mat &m);
};

