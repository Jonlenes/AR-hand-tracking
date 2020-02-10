#include "HandRegion.h"

HandRegion::HandRegion()
{
}

Mat HandRegion::getHandRegion(Mat &srcImage)
{

	Mat image;

	try {

		//Converte para o esquema HSV
		cvtColor(srcImage, image, COLOR_BGR2HSV);

		//Faz filtro pela cor preta
		inRange(image, Scalar(0, 0, 0), Scalar(180, 255, 30), image);

		//suaviza a imagem usando um kernel (ver kernel na documenta��o da opencv)
		blur(image, image, Size(5, 5));

		// Aplica um limiar de n�vel fixo para cada elemento da matriz
		// Converte a imagem para bin�ria.
		threshold(image, image, 128, 255, CV_THRESH_BINARY);

		//-----------------------------------------------------
		// Morphology Operation (Opening and Closing)

		// Faz uma eros�o seguida de uma dilata��o - Remove ruido  cv
		morphologyEx(image, image, CV_MOP_OPEN, 0);

		//Faz uma dilata��o seguida de uma eros�o -  
		// fecha pequenos orif�cios dentro dos objetos  
		morphologyEx(image, image, CV_MOP_CLOSE, 0);
		//-----------------------------------------------------
	
	}
	catch (exception &e) {
		throw ArException("HandRegion", "getHandRegion", e);
	}

	return image;
}
