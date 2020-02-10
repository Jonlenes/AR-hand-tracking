#pragma once

#include <vector>
#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

/*
Em matem�tica , os quatern�es s�o um sistema num�rico que amplia os n�meros complexos.
Eles foram descritos pela primeira vez pelo matem�tico irland�s William Rowan Hamilton 
em 1843 [1] [2] e aplicados � mec�nica no espa�o tridimensional . Uma caracter�stica 
dos quatern�es � que a multiplica��o de dois quatern�es n�o � comutativa. Hamilton 
definiu um quaternion como o quociente de duas linhas direcionadas em um espa�o 
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

