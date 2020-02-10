#include "Capture.h"

Capture::Capture()
{

}


Capture::~Capture()
{

}

bool Capture::initialize(int index) 
{
	//Abra a camera do indice passado 
	cap.open(index);

	//Verifica se conseguiu abrir a camera
	if (!cap.isOpened())
		return false;

	//Definindo a altura e a largura 
	cap.set(CV_CAP_PROP_FRAME_WIDTH, CAPTURE_SIZE_WIDTH);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, CAPTURE_SIZE_HEIGHT);

	return true;
}

Mat &Capture::nextFrame()
{
	cap >> frame;

	if (frame.empty())
		throw("Frame vazio.");

	// Invertendo a imagem, pois a opencv e opengl armazena imagem de forma oposta
	flip(frame, frame, 0); 

	return frame;
}

int Capture::getFrameWidth() const
{
	return CAPTURE_SIZE_WIDTH;
}

int Capture::getFrameHeigth() const
{
	return CAPTURE_SIZE_HEIGHT;
}
