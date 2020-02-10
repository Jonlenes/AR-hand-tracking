#pragma once

#include <opencv2\opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

class FingertipCandidate
{
public:
	FingertipCandidate();
	FingertipCandidate(Point2f point, int score, int age, 
						int lost, pair<float, float> velocity, 
						float dist);

	~FingertipCandidate();

	Point2f					_point;			    // Posi��o dos candidatos
	int						_score;				// Quatidade de vezes que foi encontrado padr�o desde ponto com um ponto do pr�ximo freme 
	int						_age;				// Tempo de vida (Quatidade de vezes que esse ponto est� na lista ou passou pelo processamento)
	int						_lost;				// Quatidade de vezes que o ponto foi perdido
	pair<float, float>		_velocity;
	float					_dist;
};

