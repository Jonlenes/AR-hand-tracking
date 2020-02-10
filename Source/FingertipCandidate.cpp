#include "FingertipCandidate.h"


FingertipCandidate::FingertipCandidate()
{
}

FingertipCandidate::FingertipCandidate(Point2f point, int score, int age,
	int lost, pair<float, float> velocity,
	float dist)
{
	_point = point;
	_score = score;
	_age = age;
	_lost = lost;
	_velocity = velocity;
	_dist = dist;
}

FingertipCandidate::~FingertipCandidate()
{
}
