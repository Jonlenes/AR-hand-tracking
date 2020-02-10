#include "FingertipPoseEstimation.h"

#define atd at<double>


FingertipPoseEstimation::FingertipPoseEstimation()
{
	_matFingerRotation = Mat::zeros(3, 1, DataType<double>::type);
	_matFingerTranslation = Mat::zeros(3, 1, DataType<double>::type);
	_matFingerRotation3by3 = Mat::zeros(3, 3, DataType<double>::type);
	_matFingerQuaternion = Mat::zeros(4, 1, DataType<double>::type);
	
	_matCameraCenterT = Mat::zeros(3, 1, DataType<double>::type);
	_matCameraCenterR = Mat::zeros(3, 3, DataType<double>::type);

	_kalmanFilter = 0;

	_fpTime.open("C:\\Users\\asus\\Desktop\\time.txt", fstream::app);

}

bool FingertipPoseEstimation::initialize(string calibFilename, Size size)
{
	// Armazenando o size
	_size = size;

	// Inicializando os modulos
	_fingerTipProcess.initialize(_size);


	if (_poseEstimation.readIntrinsicParameters(calibFilename) == false)
		return false;
	
	//
	// Texture for Video Frame
	//
	glGenTextures(1, &_nCameraTexID);
	glBindTexture(GL_TEXTURE_2D, _nCameraTexID);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	// Set texture clamping method
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP); //MY
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP); //MY
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1024, 1024, 0, GL_RGB, GL_UNSIGNED_BYTE, 0); 

	//
	// Kalman filter
	//
	_kalmanFilter = 0;
	
	
	_pKalmanMat_A = Mat::zeros(14, 14, DataType<double>::type);
	setIdentity(_pKalmanMat_A);
	for (int i = 0; i < 7; i++)
		_pKalmanMat_A.atd(i, i + 7) = 1;

	_pKalmanMat_G = Mat::zeros(14, 7, DataType<double>::type);
	for (int i = 0; i < 7; i++)
		_pKalmanMat_A.atd(i + 7, i) = 1;

	_pKalmanMat_H = Mat::zeros(7, 14, DataType<double>::type);
	for (int i = 0; i < 7; i++)
		_pKalmanMat_A.atd(i, i) = 1;;

	_pKalmanMat_W = Mat::zeros(14, 1, DataType<double>::type);

	return true;

}

void FingertipPoseEstimation::terminate()
{
	if (_kalmanFilter) {
		delete _kalmanFilter;
		_kalmanFilter = 0;
	}
}

void FingertipPoseEstimation::reset()
{
	_fingertipDetected = false;
	_fingertipTracker.reset();
	
	// Reset Kalman filter
	if (_kalmanFilter)
	{
		delete _kalmanFilter;
		_kalmanFilter = 0;
	}

}

void FingertipPoseEstimation::OnCapture(Mat frame)
{
	_image = frame;

}

void FingertipPoseEstimation::OnProcess()
{
	//Segmentação 
	Mat handImg = _handRegion.getHandRegion(_image);

	//
	// Find Fingertip Candidates
	//
	if (_fingertipDetected == false)
		_fingerTipProcess.reset(); // Limpa os vetores 

	// Retorna a quantidade de pontos de dedos encontradas 
	// (Encontra no máximo 5 candidatos de ser ponta de dedo)
	int nFingertipCandidates = _fingerTipProcess.findFingerTipCandidatesByCurvature(handImg);

	//
	// Pega Centroid
	//
	_prevCentroid = _currCentroid;
	_currCentroid = _fingerTipProcess._maxDistPoint;


	//
	// Detectar / rastrear ponto de dedos
	//
	if (_fingertipDetected == false) 
	{
		
		// Ainda n�o detectado. Tenta detectar 5 pontas nos dedos.
		// Detecta aos pontos dos dedos 
		_fingertipDetected = 
			_fingertipTracker.feedFingertipCandidates(_fingerTipProcess._vetEndPoints,
			_fingerTipProcess._vetEndPointDist, 
			_currCentroid);

	}
	else
	{
		// Detected.

		// Predict by Kalman filter
		if (_kalmanFilter) 
			Mat prediction = _kalmanFilter->predict();

		// Rastreamento dos dedos
		_fingertipDetected = _fingertipTracker.trackFingertips(_fingerTipProcess._vetEndPoints, 
			_fingerTipProcess._vetEndPointDist,
			_prevCentroid, _currCentroid);
			
	}

	// Zerando as matrizes: _matFingerRotation, _matFingerTranslation e _matFingerQuaternion
	if (_fingertipDetected == false)
		zerosMatCameraParam();

	//
	// Calcula a posi��o estimada para os dedos - Pose Estimation from Fingertips
	//
	bool fPrevPoseEstimated = _poseEstimatedByFingertips;
	bool fForceOrientation = false;
	_poseEstimatedByFingertips = false;


	if (_fingertipDetected)
	{
		// Armazena os par�metros atuais da camera
		float rotation_old[3];
		float translation_old[3];
		float quaternion_old[4];
		float center_old[3];

		for (int i = 0; i < 3; i++)
		{
			rotation_old[i] = _matFingerRotation.atd(i);
			translation_old[i] = _matFingerTranslation.atd(i);
			quaternion_old[i] = _matFingerQuaternion.atd(i);
			center_old[i] = _matCameraCenterT.atd(i);
		}
		quaternion_old[3] = _matFingerQuaternion.atd(3);
		
		// Zerando as matrizes: _matFingerRotation, _matFingerTranslation e _matFingerQuaternion
		zerosMatCameraParam();

		//cout << _matFingerRotation << endl; DEBUG

		// Busca a rotate e o transalete da m�o 
		_poseEstimatedByFingertips = _fingertipTracker.findExtrinsicCameraParams(_poseEstimation._cameraIntrinsic, 
			_poseEstimation._cameraDistortion, 
			_matFingerRotation,
			_matFingerTranslation);

		//
		// Check Camera Center Range
		//
		Rodrigues(_matFingerRotation, _matFingerRotation3by3);
			
		// Converte matiz para quaternion
		Quaternion::matrix2Quaternion(_matFingerRotation3by3, _matFingerQuaternion);
		
		// Encontra o inverso ou pseudo-inverso de uma matriz.
		invert(_matFingerRotation3by3, _matCameraCenterR);
		_matCameraCenterT = _matCameraCenterR * _matFingerTranslation;

			
		// Alterando a escala
		_matCameraCenterT.convertTo(_matCameraCenterT, -1, -1);
		cout << "\n\n_CameraCenterTMat cvScale\n" << _matCameraCenterT << endl;
		
		// Calcula a diferença de Quaternion
		float diffQuaternion = 0;
		for (int i = 0; i < 4; i++)
			diffQuaternion += pow(_matFingerQuaternion.atd(i) - quaternion_old[i], 2);
		diffQuaternion = sqrt(diffQuaternion);

		// Verifica se a camera é valida
		if (abs(_matCameraCenterT.atd(2)) > 2000.0 ||
			(_fingertipTracker.getFlipOrder() == false && _matCameraCenterT.atd(2) > 0) ||
			(_fingertipTracker.getFlipOrder() == true && _matCameraCenterT.atd(2) < 0) ||
			(center_old[0] || center_old[1] || center_old[2]) &&
			sqrt(pow(_matCameraCenterT.atd(0) - center_old[0], 2) +
			pow(_matCameraCenterT.atd(1) - center_old[1], 2) +
			pow(_matCameraCenterT.atd(2) - center_old[2], 2)) > 200)
		{
			printf("Invalid Camera (by Fingertip) (%5.2f, %5.2f, %5.2f)\n", _matCameraCenterT.atd(0), _matCameraCenterT.atd(1), _matCameraCenterT.atd(2));
			_poseEstimatedByFingertips = false;
		}
		else
			printf("OK Camera (by Fingertip) (%5.2f, %5.2f, %5.2f) diffQ = %5.2f\n", _matCameraCenterT.atd(0), _matCameraCenterT.atd(1), _matCameraCenterT.atd(2), diffQuaternion);
		
		printf("T(%5.2f %5.2f %5.2f) Q(%5.2f %5.2f %5.2f %5.2f)\n",
			_matFingerTranslation.atd(0), _matFingerTranslation.atd(1), _matFingerTranslation.atd(2),
			_matFingerQuaternion.atd(0), _matFingerQuaternion.atd(1), _matFingerQuaternion.atd(2), _matFingerQuaternion.atd(3));


		// Quando a orientação muda completamente (por exemplo, -0,97 -> 0,98),
		// forçar a orientação sem filtragem do kalman
		if ((center_old[0] || center_old[1] || center_old[2]) && diffQuaternion > 1.0 && _kalmanFilter)
		{
			fForceOrientation = true;
			for (int i = 0; i < 4; i++)
			{
				_kalmanFilter->statePre.at<float>(i + 3) = _matFingerQuaternion.atd(i);
				_kalmanFilter->statePre.at<float>(i + 10) = 0;
			}
		}

		// Voltando os par�metros antigo
		if (_poseEstimatedByFingertips == false) 
		{
			{
				for (int i = 0; i < 3; i++)
				{
					_matFingerRotation.atd(i) = rotation_old[i];
					_matFingerTranslation.atd(i) = translation_old[i];
					_matFingerQuaternion.atd(i) = quaternion_old[i];
				}
				_matFingerQuaternion.atd(3) = quaternion_old[3];
				/**/
				printf("forced to be old pose.\n");
			}
		}
		else
		{
			//
			// Init Kalman filter if necessary
			if (!_kalmanFilter)
			{
				_kalmanFilter = new KalmanFilter(14, 7, 0);
				
				for (int i = 0; i < 7; i++)
					_pKalmanMat_A.atd(i, i + 7) = 1;

				// Setando a matriz _pKalmanMat_A para a transitionMatrix
				_pKalmanMat_A.copyTo(_kalmanFilter->transitionMatrix);

				// Setando identidade
				setIdentity(_kalmanFilter->measurementMatrix, cvRealScalar(1));
				
				// noise covariances
				_kalmanFilter->processNoiseCov.setTo(Scalar::all(0));
				_kalmanFilter->measurementNoiseCov.setTo(Scalar::all(0));

				for (int i = 0; i < 3; i++)
				{
					// translation part
					_kalmanFilter->processNoiseCov.at<float>(i, i) = 10;
					_kalmanFilter->processNoiseCov.at<float>(i + 7, i + 7) = 10;
					_kalmanFilter->measurementNoiseCov.at<float>(i, i) = 30;
				}
				for (int i = 0; i < 4; i++)
				{
					// rotation part
					_kalmanFilter->processNoiseCov.at<float>(i + 3, i + 3) = 0.2;
					_kalmanFilter->processNoiseCov.at<float>(i + 10, i + 10) = 0.2;
					_kalmanFilter->measurementNoiseCov.at<float>(i + 3, i + 3) = 0.5;
				}

				// Setando identidade
				setIdentity(_kalmanFilter->errorCovPost, cvRealScalar(1));

				// Zerando statePost
				_kalmanFilter->statePost.setTo(Scalar::all(0));

				for (int i = 0; i < 3; i++)
					_kalmanFilter->statePost.at<float>(i) = _matFingerTranslation.atd(i);
			
				for (int i = 0; i < 4; i++)
					_kalmanFilter->statePost.at<float>(i + 3) = _matFingerQuaternion.atd(i);
			}
			else
			{
				// Kalman correct
				float measurement[7];
				Mat measurementMat(7, 1, CV_32FC1, measurement);

				// Salvado o translation
				for (int i = 0; i < 3; i++)
					measurement[i] = _matFingerTranslation.atd(i);
				
				// Salvando o Quaternion
				for (int i = 0; i < 4; i++)
					measurement[i + 3] = _matFingerQuaternion.atd(i);
				
				// update A's delta T
				//for (int i = 0; i < 7; i++)
				//	cvSetReal2D(_pKalmanMat_A, i, i + 7, _deltaT);
				
				// Copiando
				_pKalmanMat_A.copyTo(_kalmanFilter->transitionMatrix);
				//cvCopy(_pKalmanMat_A, _pKalman->transition_matrix);

				// correct kalman filter state
				_kalmanFilter->correct(measurementMat);

			}

			// Copy the corrected state
			for (int i = 0; i < 3; i++)
				_matFingerTranslation.atd(i) = _kalmanFilter->statePost.at<float>(i);

			for (int i = 0; i < 4; i++)
				_matFingerQuaternion.atd(i) = _kalmanFilter->statePost.at<float>(i + 3); 
		}
		
	}
	else
	{
		// Reset Kalman filter
		if (_kalmanFilter)
		{
			delete _kalmanFilter;
			_kalmanFilter = 0;
		}
	}

	_validPose = _poseEstimatedByFingertips || (_fingertipDetected && fPrevPoseEstimated);
	printf("pose: %s\n", _validPose ? "valid" : "invalid");

	//
	// Compute Camera Center
	//

	printf("T(%5.2f %5.2f %5.2f) Q(%5.2f %5.2f %5.2f %5.2f)\n",
		_matFingerTranslation.atd(0), _matFingerTranslation.atd(1), 
		_matFingerTranslation.atd(2), _matFingerQuaternion.atd(0), 
		_matFingerQuaternion.atd(1), _matFingerQuaternion.atd(2), 
		_matFingerQuaternion.atd(3));

	// Converte matiz para quaternion (!)
	Quaternion::quaternion2Matrix(_matFingerQuaternion, _matFingerRotation3by3);
	
	// Converte uma matriz de rota��o em um vetor de rota��o ou vice-versa.
	Rodrigues(_matFingerRotation3by3, _matFingerRotation);

	invert(_matFingerRotation3by3, _matCameraCenterR);

	// _cameraCenterT =  _cameraCenterR * _vetFingerTranslation
	_matCameraCenterT = _matCameraCenterR * _matFingerTranslation;
		
	// Alterando a escala
	_matCameraCenterT.convertTo(_matCameraCenterT, -1, -1);
		
	// Mostrando os par�metros da camera
	printf("Camera(by Fingertips) = (%5.2f, %5.2f, %5.2f)\n", _matCameraCenterT.atd(0), _matCameraCenterT.atd(1), _matCameraCenterT.atd(2));
}

//
// Faz desenhos usando a posi��o da m�o e dos dedos 
//
void FingertipPoseEstimation::OnDisplay()
{
	// Se os dedos n�o foi detectado
	if (_image.empty()) 
		return;

	//
	// Fingertips ( Candidates )
	//
	Scalar color[] = {
		{ Scalar( 64,    64,    255 ) },	// red
		{ Scalar( 0,     128,   255 ) },	// orange
		{ Scalar( 0,     255,   255 ) },	// yellow
		{ Scalar( 0,     192,   0   ) },	// green
		{ Scalar( 255,   0,     0   ) }     // blue
	};

	// Desenhando circulos nas pontas dos dedos
	for (int i = 0; i < NUM_FINGERTIP; i++)
	{
		FingerTip *fingerTip = _fingertipTracker.getFingertip(i);
		if (fingerTip)
			circle(_image, fingerTip->_point, fingerTip->_dist, color[i], -1);
	}

	// Desenhando as elipses no dedo
	for (int i = 0; i < int(_fingerTipProcess._vetEndPoints.size()); i++)
	{
		Point2f center = _fingerTipProcess._ellipses[i].center;
		Size2f size = cvSize(_fingerTipProcess._ellipses[i].size.width * 0.5, 
			_fingerTipProcess._ellipses[i].size.height * 0.5);

		if (size.width > 0 && size.height > 0 && size.width < _size.width && size.height < _size.height)
			//ellipse(_image, _fingerTipProcess._ellipses[i], Scalar(0, 255, 0), 1, CV_AA);
			ellipse(_image, center, size, 
			_fingerTipProcess._ellipses[i].angle, 
			0, 360, Scalar(0, 255, 0));
	}

	//
	// Max Distance Point + Region of Interest
	//
	circle(_image, _fingerTipProcess._maxDistPoint,
		_fingerTipProcess._maxDistValue * 0.7, 
		Scalar(255, 255, 0), 1, 8, 0);

	if (_validPose) 
		putText(_image, "OK", Point(100, 300), FONT_HERSHEY_DUPLEX, 1, Scalar(0, 0, 255));

}



//Método original da HandyAR
void FingertipPoseEstimation::setOpenGLFrustrum()
{
	if (_image.empty())
		return;

	//set viewing frustrum to match camera FOV (ref: ARTag's 3d_augmentations.cpp)
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	double camera_fx = _poseEstimation._cameraIntrinsic.atd(0, 0);
	double camera_fy = _poseEstimation._cameraIntrinsic.atd(1, 1);
	double camera_ox = _poseEstimation._cameraIntrinsic.atd(0, 2);
	double camera_oy = _poseEstimation._cameraIntrinsic.atd(1, 2);
	double camera_opengl_dRight = (double)( _size.width - camera_ox) / (double)(camera_fx);
	double camera_opengl_dLeft = -camera_ox / camera_fx;
	double camera_opengl_dTop = (double)( _size.height - camera_oy) / (double)(camera_fy);
	double camera_opengl_dBottom = -camera_oy / camera_fy;
	
	//if (_validPose) 
		glFrustum(camera_opengl_dLeft, camera_opengl_dRight, camera_opengl_dBottom, camera_opengl_dTop, 1.0, 102500.0);
	
	camera_opengl_dLeft *= 10240;
	camera_opengl_dRight *= 10240;
	camera_opengl_dBottom *= 10240;
	camera_opengl_dTop *= 10240;

	// convert image R and B channel
	cvtColor(_image, _image, CV_RGB2BGR);

	// draw the camera frame
	glBindTexture(GL_TEXTURE_2D, _nCameraTexID);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, _size.width, _size.height, GL_RGB, GL_UNSIGNED_BYTE, _image.data);
	
	glClearColor(0, 0, 0, 0);

	// draw a quad for the background video
	glMatrixMode(GL_TEXTURE);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glDisable(GL_LIGHTING);

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, _nCameraTexID);

	glBegin(GL_QUADS);
	glColor3f(1.0f, 1.0f, 1.0f);
	//draw camera texture, set with offset to aim only at cam_width x cam_height upper left bit
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(camera_opengl_dLeft, camera_opengl_dBottom, -10240);
	glTexCoord2f(0.0f, (float)_size.height / 1024.0);
	glVertex3f(camera_opengl_dLeft, camera_opengl_dTop, -10240);
	glTexCoord2f((float)_size.width / 1024.0, (float)_size.height / 1024.0);
	glVertex3f(camera_opengl_dRight, camera_opengl_dTop, -10240);
	glTexCoord2f((float)_size.width / 1024.0, 0.0f);
	glVertex3f(camera_opengl_dRight, camera_opengl_dBottom, -10240);
	glEnd();

	glDisable(GL_TEXTURE_2D);
	glEnable(GL_LIGHTING);
}

//Método original da HandyAR
void FingertipPoseEstimation::setOpenGLModelView()
{
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt(
		_matCameraCenterT.atd(0),
		_matCameraCenterT.atd(1),
		-_matCameraCenterT.atd(2),
		_matCameraCenterT.atd(0) + _matFingerRotation3by3.atd(2, 0),
		_matCameraCenterT.atd(1) + _matFingerRotation3by3.atd(2, 1),
		-(_matCameraCenterT.atd(2) + _matFingerRotation3by3.atd(2, 2)),
		_matFingerRotation3by3.atd(1, 0),
		_matFingerRotation3by3.atd(1, 1),
		-_matFingerRotation3by3.atd(1, 2));

	glGetFloatv(GL_MODELVIEW_MATRIX, _modelView);
}

bool FingertipPoseEstimation::loadFingertipCoordinates(string filename)
{
	return _fingertipTracker.loadFingertipCoordinates(filename);
}

void FingertipPoseEstimation::zerosMatCameraParam()
{
	_matFingerRotation.setTo(Scalar::all(0));
	_matFingerTranslation.setTo(Scalar::all(0));
	_matFingerQuaternion.setTo(Scalar::all(0));
}