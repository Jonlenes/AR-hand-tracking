#pragma once

#include <opencv2\opencv.hpp>
#include <vector>

#include "ArException.h"

using namespace cv;
using namespace std;

#define MAX_END_POINT				1000

#define MIN_CURVATURE_STEP          10
#define MAX_CURVATURE_STEP          50
#define FINGERTIP_ANGLE_THRESHOLD   0.5
#define GAP_POINT_THRESHOLD         10
#define CONNECTED_POINT_THRESHOLD   10

class FingertipProcess
{
public:
	FingertipProcess();

	void reset();

	void initialize(Size size);
	int findFingerTipCandidatesByCurvature(Mat handRegion);

public: 

	void getMaxDistPoint(Mat &mat, Mat *output);
	void clearVet();

	// Tamanho das imagens que serão processadas
	Size _size;

	// Pontos finais (Candidatos a ponto de dedo)
	vector<Point2f>   _vetEndPoints;
	vector<float>     _vetEndPointDist;
	vector<float>     _vetEndPointScore;
	
	// Armazena um componente conectado para a mão
	Mat				_ccContourImage;

	// Transformação da distancia
	Point			_maxDistPoint;
	double          _maxDistValue;
	Mat				_distHandImage; // DTI
	Mat				_distImage; // TDTI

	// Ellipses
	vector<RotatedRect> _ellipses;
};

