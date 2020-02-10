#include "FingertipProcess.h"

FingertipProcess::FingertipProcess()
{
}

void FingertipProcess::reset()
{
	_vetEndPoints.clear();
	_vetEndPointDist.clear();
	_vetEndPointScore.clear();
	_ellipses.clear();
}

void FingertipProcess::initialize(Size size)
{
	_size = size;
	_ccContourImage = Mat::zeros(_size, CV_8UC1);
	_distImage = Mat::zeros(_size, IPL_DEPTH_32F);
	_distHandImage = Mat::zeros(_size, IPL_DEPTH_32F);
}

//
// Aplica a transformada da distancia e busca o ponto com distancia máxima
//
void FingertipProcess::getMaxDistPoint(Mat &mat, Mat *output)
{
	// Funcão da opencv que calcula a tranformada da distancia
	distanceTransform(mat, *output, CV_DIST_L2, CV_DIST_MASK_3);
	
	// Esse ponto, consequentimente será o centroid da mão
	_maxDistValue = 0;
	_maxDistPoint = cvPoint(0, 0);

	// Pega o valor maximo de _pDistImage2 e o ponto que tem esse valor máximo
	minMaxLoc(*output, NULL, &_maxDistValue, NULL, &_maxDistPoint);

	// Ajustando os limites
	if (_maxDistPoint.x == 0) _maxDistPoint.x++;
	if (_maxDistPoint.y == 0) _maxDistPoint.y++;
	if (_maxDistPoint.x >= _size.width - 1) _maxDistPoint.x--;
	if (_maxDistPoint.y >= _size.height - 1) _maxDistPoint.y--;
}

int FingertipProcess::findFingerTipCandidatesByCurvature(Mat handRegion)
{
	reset();
	
	if (handRegion.empty()) 
		return 0;

	// Se não tem pontos finais
	if (!_vetEndPoints.size()) {
		// Encontro o ponto com distancia máxima da borda
		getMaxDistPoint(handRegion, &_distImage);
		//Util::savePrint("8_dist_trans", _distImage);
	}

	//
	// Encontrar contornos para obter um único componente conectado para a 
	// mão que possui o ponto de distância máximo nele.
	//
	vector< vector<Point> > contours;
	int i;
	
	// Localiza contornos em uma imagem binária.
	findContours(handRegion, contours, CV_RETR_EXTERNAL, 
		CV_CHAIN_APPROX_SIMPLE, Point(0, 0));


	// Verifico se foi encontrado algum contrno
	if (contours.empty()) return 0;

	//Limpando a matriz que armazenará os contornos
	_ccContourImage.setTo(Scalar::all(0));

	// Percorre todos os contornos encontrados - 
	// Um contorno é determinando por um conjunto de pontos
	for (i = 0; i < int(contours.size()); ++i) {

		// Pego o contorno atual
		auto contour = contours[i];

		// Se o contorno tem mais de 100 pontos
		if (contour.size() <= 100) continue;
			
		// Desenha o contorno e faz o preenchimento - CUIDADO
		drawContours(_ccContourImage, contours, i, Scalar(255), CV_FILLED);

		// Verifico se o ponto maximo (centro da mão) foi "pintado" por este contorno
		if (int(_ccContourImage.at<uchar>(_maxDistPoint)) == 255) {
			
			// Limpando os contornos, para redesenhar apenas o contorno atual
			_ccContourImage.setTo(Scalar::all(0));
				
			// Desenha o contorno atual novamente 
			drawContours(_ccContourImage, contours, i, Scalar(255), CV_FILLED);

			// Calcula o ponto de maior distancia dentro da imagem
			getMaxDistPoint(_ccContourImage, &_distHandImage);

			break;
		}

	}

	// Se não achou contorno
	if (i == int(contours.size()) || contours[i].size() < 100)
		return 0;

	//
	// Encontrar pontos máximos curvatura local
	//
	
	// Variaveis utilizadas
	int nPointsConnected = 0;
	int nPointsGap = 0;
	double minAngle = 0;
	int   minPointIndex = -1;
	double mediumIndex = 0;
	double sumAngle = 0;
	double meanPointX = 0;
	double meanPointY = 0;
	int min_curvature_step = int(_maxDistValue * 0.2);
	int max_curvature_step = int(_maxDistValue * 1.0);
	if (min_curvature_step < MIN_CURVATURE_STEP) 
		min_curvature_step = MIN_CURVATURE_STEP;
	if (max_curvature_step > MAX_CURVATURE_STEP) 
		max_curvature_step = MAX_CURVATURE_STEP;
	int startIndex, endIndex;

	// Contorno escolhido 
	vector< Point > &contour = contours[i];
	int countPoints = int(contour.size());

	// Percorrento os pontos novamente
	for (i = 0; i < countPoints; ++i)
	{
		//
		// For different scale, calculate curvature
		// _maxDistValue is approximately our bound.
		//
		double minK = 0;
		double minDir = 0;
		double count = 0;

		for (int k = min_curvature_step; k <= max_curvature_step; k++)
		{
			int iPrev = (i - k + countPoints) % countPoints;
			int iNext = (i + k) % countPoints;

			// Compute Curvature
			int pp1_x = contour[iPrev].x - contour[i].x;
			int pp1_y = contour[iPrev].y - contour[i].y;
			int pp2_x = contour[iNext].x - contour[i].x;
			int pp2_y = contour[iNext].y - contour[i].y;

			// dot product ( cosine )
			float cross = (pp1_x*pp2_x + pp1_y*pp2_y)
				/ (sqrt((float)pp1_x*pp1_x + pp1_y*pp1_y) *
				sqrt((float)pp2_x*pp2_x + pp2_y*pp2_y));
			// cross product ( sign of z-component )
			double dir = pp1_x*pp2_y - pp1_y*pp2_x;

			if (minK <= cross)
			{
				minK = cross;
				minDir = dir;
			}
		}

		// Tomar os pontos com base na curvatura e direção
		if (minK > FINGERTIP_ANGLE_THRESHOLD && minDir > 0)
		{
			nPointsGap = 0;
			if (nPointsConnected == 0) startIndex = i;
			nPointsConnected++;
			if (minAngle < minK)
			{
				minAngle = minK;
				minPointIndex = i;
			}
			mediumIndex += (minK * i);
			meanPointX += (minK * contour[i].x);
			meanPointY += (minK * contour[i].y);
			sumAngle += minK;
		
		} else {

			nPointsGap++;
			if (nPointsGap >= GAP_POINT_THRESHOLD && nPointsConnected > CONNECTED_POINT_THRESHOLD)
			{
				// check image boundary
				if (contour[minPointIndex].x > 10 &&
					contour[minPointIndex].x < _size.width - 10 &&
					contour[minPointIndex].y > 10 &&
					contour[minPointIndex].y < _size.height - 10)
				{
					mediumIndex /= sumAngle;
					meanPointX /= sumAngle;
					meanPointY /= sumAngle;

					// Armazenando o ponto da ponta de dedo
					_vetEndPoints.push_back( Point2f(float(meanPointX), float(meanPointY) ) );
						
					endIndex = i;


					// Adapta-se a uma elipse em torno de um conjunto de pontos 2D
					_ellipses.push_back( fitEllipse(vector<Point>(contour.begin() + startIndex, 
						contour.begin() + endIndex)) );

					// Encontre o ponto final ao longo do primeiro eixo
					double minDist = 1000000;
					for (int dAngle = 90; dAngle < 360; dAngle += 180) {
							
						// Pegando a ultima elipse
						RotatedRect ellipse = _ellipses[ _ellipses.size() - 1];

						// Coordenadas do centro da elipse
						double x1 = ellipse.center.x;
						double y1 = ellipse.center.y;
							
						if (dAngle == 0 || dAngle == 180)
						{
							x1 += cos( (ellipse.angle + dAngle) * CV_PI / 180 ) * ellipse.size.width * 0.5;
							y1 += sin( (ellipse.angle + dAngle) * CV_PI / 180 ) * ellipse.size.width * 0.5;
						}
						else
						{
							x1 += cos( (ellipse.angle + dAngle) * CV_PI / 180) * ellipse.size.height * 0.5;
							y1 += sin( (ellipse.angle + dAngle) * CV_PI / 180) * ellipse.size.height * 0.5;
						}
						double dist = pow(meanPointX - x1, 2) + pow(meanPointY - y1, 2);
							
						if (minDist > dist) minDist = dist;
					}

					_vetEndPointDist.push_back(10);
					_vetEndPointScore.push_back(1);

					if (int(_vetEndPoints.size()) == MAX_END_POINT)
						break;

				}
			}

			if (nPointsGap >= GAP_POINT_THRESHOLD)
			{

				nPointsConnected = 0;
				minPointIndex = -1;
				minAngle = 0;
				mediumIndex = 0;
				meanPointX = 0;
				meanPointY = 0;
				sumAngle = 0;
			}
		}
	}

	return int(_vetEndPoints.size());
}
