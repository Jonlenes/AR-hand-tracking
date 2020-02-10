#pragma once

#include <opencv2\opencv.hpp>
#include <GL\glut.h>

#include <iomanip>

#include "Quaternion.h"

#include "HandRegion.h"
#include "FingertipProcess.h"
#include "FingertipTracker.h"
#include "PoseEstimation.h"

using namespace cv;

class FingertipPoseEstimation
{
public:
	FingertipPoseEstimation();
	~FingertipPoseEstimation();

	bool initialize(string calibFilename, Size size);
	void terminate();
	void reset();

	void OnCapture(Mat frame);
	void OnProcess();
	void OnDisplay();

	bool loadFingertipCoordinates(string filename);

	// Interação da openCL para openGL
	void setOpenGLFrustrum();   //Frustrum de visualização
	void setOpenGLModelView();  //Posição do observador 

private:
	//
	// Tamanho das imagens que serão processadas
	//
	Size _size;

	//
	// Imagens
	//
	Mat _image;
	Mat _grayImage;

	//
	// Modulos de processamento
	//
	HandRegion _handRegion;
	FingertipProcess _fingerTipProcess;
	FingertipTracker _fingertipTracker;
	PoseEstimation _poseEstimation;

	//
	// Centro da m�o
	//
	Point _prevCentroid;
	Point _currCentroid;

	//
	// Running Mode
	//
	bool    _fingertipDetected;

	//
	// Pose Estimation by Fingertips
	//
	bool					_poseEstimatedByFingertips;
	bool					_validPose;
	Mat						_matFingerRotation;
	Mat						_matFingerRotation3by3;
	Mat						_matFingerTranslation;
	Mat						_matFingerQuaternion;
	
	//
	// Camera Transform
	//
	Mat			_matCameraCenterT;
	Mat			_matCameraCenterR;

	//
	// OpenGL View Frustrum
	//
	GLuint  _nCameraTexID;

	//
	// OpenGL ModelView Matrix
	//
	float   _modelView[16];

	//
	// Kalman filter for the camera pose
	// x(t+1) = A x(t) + G w(t)
	// y(t)   = H x(t) +   v(t)
	//
	KalmanFilter *_kalmanFilter;
	Mat           _pKalmanMat_A;  // State Transition Matrix
	Mat           _pKalmanMat_G;  // Driving Matrix
	Mat           _pKalmanMat_H;  // Observation Matrix
	Mat           _pKalmanMat_W;  // Process Noise Matrix

    //
    // Processing Time Profile
    //
    fstream  _fpTime;
    int64   _PrevTickCount;



	// Metodos privados
	void zerosMatCameraParam();
};

