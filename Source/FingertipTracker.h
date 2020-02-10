#pragma once

#include <opencv2\opencv.hpp>
#include <algorithm>
#include <fstream>

#include "FingertipCandidate.h"
#include "FingerTip.h"

#include "ArException.h"

using namespace cv;
using namespace std;

// Numero máximo de candidato a dedo
#define NUM_MAX_CANDIDATES          20			
// Distancia maxima entre o candidato 
// perdido e um novo canditado para ser procurar padrão
#define THRESHOLD_CLOSEST_DIST      50
// Quantidade de vezes que o ponto pode ser 
// perdido e continuar na lista de candidatos
#define THRESHOLD_LOST_TRACKING     5
// Quantidade de vezes que deve ser encontrado 
// se manter no vetor de candidatos
#define MIN_AGE_TO_TRACK            5
// Quantidade de vezes que deve ser encontrado 
//na imagem para ser detectado
#define THRESHOLD_AGE_TO_DETECT     10          

#define NUM_FINGERTIP               5

#define MODE_FINGERTIP_NONE				0
#define MODE_FINGERTIP_FIRST_DETECTED   1
#define MODE_FINGERTIP_TRACKING         2
#define MODE_FINGERTIP_LOST_TRACKING    3


class FingertipTracker
{
public:
	FingertipTracker();
	~FingertipTracker();

	bool feedFingertipCandidates(vector<Point2f> &points, 
		vector<float> &distValue, 
		Point currCentroid);
	bool trackFingertips(vector<Point2f> &points, 
		vector<float> &distValue, 
		Point prevCentroid, 
		Point currCentroid);
	bool findExtrinsicCameraParams(Mat &cameraIntrinsic, 
		Mat &cameraDistortion, 
		Mat &fingerRotation, 
		Mat &fingerTranslation);

	void reset();

	bool loadFingertipCoordinates(string filename);

	bool getFlipOrder() { return _fingertipFlipOrder; }
	FingerTip * getFingertip(int index);


protected:
	void matchCorrespondencesByNearestNeighbor(vector<Point2f> &points, 
		vector<float> &distValue);

private:

	//
	// Tracking Mode
	//
	int             _nMode;

	//
	// Candidates
	//
	vector<FingertipCandidate>  _vetCandidates;


	//
	// Fingertips
	//
	bool				_detected;
	bool				_fingertipFlipOrder;
	vector<FingerTip>	_vetFingertip;
	vector<bool>		_vetTracked;

	//
	// Fingertip Coordinates
	//
	bool						_fingertipCoordinates;
	vector< Point3f >			_vetFingertipCoordinates;
	float						_numCoordinateSamples;

};

