#include "FingertipTracker.h"
#include "Util.h"

FingertipTracker::FingertipTracker()
{
}


FingertipTracker::~FingertipTracker()
{
}

void FingertipTracker::reset()
{
	_nMode = MODE_FINGERTIP_NONE;
	_detected = false;

	_vetCandidates.clear();
	_vetFingertip.clear();
}

FingerTip * FingertipTracker::getFingertip(int index)
{
	if (index >= int(_vetFingertip.size())) return 0;

	return &(_vetFingertip[index]);
}

//
// Correspondencia por vizinho mais próximo
//
void FingertipTracker::matchCorrespondencesByNearestNeighbor(vector<Point2f> &points, 
	vector<float> &distValue)
{
	int nPoints = int(points.size());
	vector<bool> matched(nPoints, false);

	for (int i = 0; i < int(_vetCandidates.size()); i++)
	{
		//
		// Encontre o ponto mais próximo de cada ponto candidato
		//
		float minDist = THRESHOLD_CLOSEST_DIST;
		int   minJ = -1;
		for (int j = 0; j < nPoints; j++)
		{
			float dist = sqrt(
				pow((float)points[j].x - _vetCandidates[i]._point.x, 2) +
				pow((float)points[j].y - _vetCandidates[i]._point.y, 2));
			if (!matched[j] && dist < minDist)
			{
				minDist = dist;
				minJ = j;
			}
		}

		if (minJ >= 0)
		{
			//
			// Se existe atualiza a pontuação e a posição 
			//
			matched[minJ] = true;

			_vetCandidates[i]._velocity.first = points[minJ].x - _vetCandidates[i]._point.x;
			_vetCandidates[i]._velocity.second = points[minJ].y - _vetCandidates[i]._point.y;

			_vetCandidates[i]._point.x = points[minJ].x;
			_vetCandidates[i]._point.y = points[minJ].y;

			_vetCandidates[i]._dist = distValue[minJ];

			_vetCandidates[i]._score++;
			_vetCandidates[i]._lost = 0;

		}
	}

	// Percorro todos os pontos 
	for (int i = 0; i < nPoints; i++)
	{
		// Se não foi encontrado o padrão e ainda não tiver no máximo possivel de candidados
		if (int(_vetCandidates.size()) < NUM_MAX_CANDIDATES && matched[i] == false)
			_vetCandidates.push_back(
			FingertipCandidate(points[i], 0, 0, 0, pair<float, float> (0.0f, 0.0f), distValue[i]));
	}

}

bool FingertipTracker::feedFingertipCandidates(vector<Point2f> &points,
											vector<float> &distValue, 
											Point currCentroid) 
{

	//
	// Basicamente incrementa o valor perdido. Quando encontrar, o valor será decrementado
	//
	for (int i = 0; i < int(_vetCandidates.size()); i++)
		_vetCandidates[i]._lost++;

	//
	//  Correspondencia por vizinho mais próximo
	//
	matchCorrespondencesByNearestNeighbor(points, distValue);

	
	//
	// Apago os pontos candidatos por valor perdido e pontuação
	//
	for (int i = 0; i < int(_vetCandidates.size()); i++)
	{
		if (_vetCandidates[i]._lost > THRESHOLD_LOST_TRACKING ||
			_vetCandidates[i]._age > MIN_AGE_TO_TRACK &&
			(float)_vetCandidates[i]._score / _vetCandidates[i]._age < 0.5)
		{
			// Removo o ponto
			_vetCandidates.erase( _vetCandidates.begin() + i );
			i--;
		}
	}

	//
	// Incrementa a idade
	//
	for (int i = 0; i < int(_vetCandidates.size()); i++)
		_vetCandidates[i]._age++;


	//
	// Faço o modelo Fingertip para a ponta dos dedos detectada
	//
	_detected = false;
	_vetFingertip.clear();
	
	// Verifica se detectou 5 pontas de dedos
	for (int i = 0; i < int(_vetCandidates.size()); i++)
	{
		if (_vetCandidates[i]._age > THRESHOLD_AGE_TO_DETECT &&
			_vetCandidates[i]._point.y > currCentroid.y)
		{
			//
			// Detected !
			//
			_vetFingertip.push_back(FingerTip(i, _vetCandidates[i]._point, _vetCandidates[i]._dist));

			if (int(_vetFingertip.size()) == NUM_FINGERTIP)
			{
				_detected = true;
				break;
			}
		}
	}

	//
	// Quando detectado, atribuir o número do índice da ponta do dedo
	//
	if (_detected) {
		
		//Faz a ordenação dos dedos (por x-axis)
		sort(_vetFingertip.begin(), _vetFingertip.end());

		//
		// Determinar o polegar com base na distância até o ponto médio
		//

		// Calcula o ponto médio 
		float mean_x = 0;
		float mean_y = 0;

		for (auto item : _vetFingertip) {
			mean_x += item._point.x;
			mean_y += item._point.y;
		}

		mean_x /= float(_vetFingertip.size());
		mean_y /= float(_vetFingertip.size());

		// Pega o dedo mais distante
		float maxDist = 0;
		int maxDistIndex = 0;
		for (int i = 0; i < int(_vetFingertip.size()); ++i) {

			auto item = _vetFingertip[i];
			float dist = sqrt( pow(mean_x - item._point.x, 2) + pow(mean_y - item._point.y, 2) );
			if (dist > maxDist) {
				maxDist = dist;
				maxDistIndex = i;
			}
		}

		// Se as distancias são negativas, então inverte os pontos.
		_fingertipFlipOrder = (maxDistIndex == 0);
		
		if (_fingertipFlipOrder)  reverse(_vetFingertip.begin(), _vetFingertip.end());
	}

	//
	// Update Mode
	//
	if (_detected)
	{
		if (_nMode == MODE_FINGERTIP_NONE)
			_nMode = MODE_FINGERTIP_FIRST_DETECTED;
		if (_nMode == MODE_FINGERTIP_LOST_TRACKING)
			_nMode = MODE_FINGERTIP_TRACKING;
	}

	return _detected;
}

bool FingertipTracker::trackFingertips(vector<Point2f> &points, 
	vector<float> &distValue,
	Point prevCentroid, Point currCentroid)
{
	try {

		//
		// Init
		//
		_vetTracked.assign(NUM_FINGERTIP, false);

		//
		// Check
		//
		if (int(points.size()) < NUM_FINGERTIP) {
			reset();
			_fmode = MODE_FINGERTIP_LOST_TRACKING;
			return _detected;
		}

		//
		// Correspondência de pontos e pontas dos dedos
		//
		int index[5] = { 0 };
		int minIndex[5] = { 0 };
		float dist[5] = { 0 };
		float minCost = 1000000;
		int nPoints = int(points.size());

		for (int pt0 = 0; pt0 < nPoints; pt0++)                    // first fingertip
		{
			index[0] = pt0 % nPoints;
			dist[0] = sqrt(
				pow((float)points[index[0]].x - currCentroid.x - _vetFingertip[0]._point.x + prevCentroid.x, 2) +
				pow((float)points[index[0]].y - currCentroid.y - _vetFingertip[0]._point.y + prevCentroid.y, 2));

			for (int pt1 = 1; pt1 <= nPoints - 4; pt1++)            // second fingertip
			{
				index[1] = _fingertipFlipOrder ? Util::mod(pt0 - pt1, nPoints) : (pt0 + pt1) % nPoints;
				dist[1] = sqrt(
					pow((float)points[index[1]].x - currCentroid.x - _vetFingertip[1]._point.x + prevCentroid.x, 2) +
					pow((float)points[index[1]].y - currCentroid.y - _vetFingertip[1]._point.y + prevCentroid.y, 2));

				for (int pt2 = 1; pt2 <= nPoints - pt1 - 3; pt2++)        // third fingertip
				{
					index[2] = _fingertipFlipOrder ? Util::mod(pt0 - pt1 - pt2, nPoints) : (pt0 + pt1 + pt2) % nPoints;
					dist[2] = sqrt(
						pow((float)points[index[2]].x - currCentroid.x - _vetFingertip[2]._point.x + prevCentroid.x, 2) +
						pow((float)points[index[2]].y - currCentroid.y - _vetFingertip[2]._point.y + prevCentroid.y, 2));

					for (int pt3 = 1; pt3 <= nPoints - pt1 - pt2 - 2; pt3++)    // fourth fingertip
					{
						index[3] = _fingertipFlipOrder ? Util::mod(pt0 - pt1 - pt2 - pt3, nPoints) : (pt0 + pt1 + pt2 + pt3) % nPoints;
						dist[3] = sqrt(
							pow((float)points[index[3]].x - currCentroid.x - _vetFingertip[3]._point.x + prevCentroid.x, 2) +
							pow((float)points[index[3]].y - currCentroid.y - _vetFingertip[3]._point.y + prevCentroid.y, 2));

						for (int pt4 = 1; pt4 <= nPoints - pt1 - pt2 - pt3 - 1; pt4++)// fifth fingertip
						{
							index[4] = _fingertipFlipOrder ? Util::mod(pt0 - pt1 - pt2 - pt3 - pt4, nPoints) : (pt0 + pt1 + pt2 + pt3 + pt4) % nPoints;
							dist[4] = sqrt(
								pow((float)points[index[4]].x - currCentroid.x - _vetFingertip[4]._point.x + prevCentroid.x, 2) +
								pow((float)points[index[4]].y - currCentroid.y - _vetFingertip[4]._point.y + prevCentroid.y, 2));

							//
							// Compute the matching cost
							//
							float matchingCost = 0;
							for (int i = 0; i < 5; i++)
							{
								matchingCost += dist[i];
							}

							//
							// Find minimum cost matching
							//
							if (matchingCost < minCost)
							{
								minCost = matchingCost;
								for (int i = 0; i < 5; i++)
								{
									minIndex[i] = index[i];
								}
							}
						}
					}
				}
			}
		}

		//
		// Update Tracking
		//
		for (int i = 0; i < 5; i++)
		{
			_vetFingertip[i]._point.x = points[minIndex[i]].x;
			_vetFingertip[i]._point.y = points[minIndex[i]].y;
			_vetFingertip[i]._dist = distValue[minIndex[i]];

			_vetTracked[i] = true;
		}

		_nMode = MODE_FINGERTIP_TRACKING;
	}
	catch (Exception &e) {
		throw ArException("FingertipTracker", "trackFingertips", e);
	}
	return _detected;
}

bool FingertipTracker::findExtrinsicCameraParams(Mat &cameraIntrinsic, 
	Mat &cameraDistortion, 
	Mat &fingerRotation, 
	Mat &fingerTranslation)
{
	bool poseEstimated = false;

	try {

		//
		// Check availability
		//
		if (int(_vetFingertip.size()) != NUM_FINGERTIP || _fingertipCoordinates == false)
			return false;

		// Preparando o vector com os pontos
		vector< Point2f > ptos;
		for (auto item : _vetFingertip)
			ptos.push_back(item._point);


		//
		// Call OpenCV function
		//

		// Essa função entroca 6 valores, 3 de rotação e 3 de tradução, Dado um modelo 2d uma imagem.
		poseEstimated = solvePnP(_vetFingertipCoordinates, 
			ptos, 
			cameraIntrinsic, 
			cameraDistortion, 
			fingerRotation, 
			fingerTranslation,
			false,
			CV_ITERATIVE);

		ptos.clear();

	}
	catch (Exception const &e) {
		throw ArException("FingertipTracker", "findExtrinsicCameraParams", e);
	}

	return poseEstimated;
}

bool FingertipTracker::loadFingertipCoordinates(string filename)
{
	//
	// Carrega as coordenadas das pontos dos dedos
	//
	fstream file(filename.c_str(), fstream::in | fstream::out);

	if (!file.is_open())
		return false;

	_vetFingertipCoordinates.assign(NUM_FINGERTIP, Point3d());

	for (int i = 0; i < NUM_FINGERTIP; i++)
	{
		file >> _vetFingertipCoordinates[i].x;
		file >> _vetFingertipCoordinates[i].y;
		file >> _vetFingertipCoordinates[i].z;
	}

	file.close();

	_fingertipCoordinates = true;

	return _fingertipCoordinates;
}