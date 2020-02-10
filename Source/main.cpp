#include <iostream>
#include <vector>

#include <opencv2\opencv.hpp>
#include <GL\glut.h>

#include "Capture.h"
#include "FingertipPoseEstimation.h"

using namespace std;
using namespace cv;

string fingertipFilename = "fingertip_640x480.dat";
string calibrationFileName = "calibration.txt";
string earthTextureFileName = "earthTexture.jpg";

int indexCamera = 1;

Capture capture;
FingertipPoseEstimation fingertipPoseEstimation;

// Render Model
#define MODEL_BUNNY				1
#define MODEL_EARTH				2
#define MODEL_MAGIC				3
#define MODEL_COORDINATE_AXES	4
#define MODEL_PHOTO_ALBUM		5
int gnModel = MODEL_EARTH;

// Earth Texture
GLuint  gnEarthTexID;

void initOpenGL();
void display();
void idle(); 


int main(int argc, char **argv) {

	try {

		//video.set(CV_CAP_PROP_FRAME_WIDTH, 256);
		//video.set(CV_CAP_PROP_FRAME_HEIGHT, 256);

		// Carregando o arquivo com as coordenandas das pontas dos dedos 
		if (fingertipPoseEstimation.loadFingertipCoordinates(fingertipFilename) == false)
			printf("fingertip coordinate '%s' file was not loaded.\n", fingertipFilename.c_str());
		else
			printf("fingertip coordinate '%s' file was loaded.\n", fingertipFilename.c_str());

		// Inicializa a captura
		if (!capture.initialize(indexCamera)) {
			cout << "Erro ao abrir a camera.";
			exit(0);
		}

		glutInit(&argc, &argv[0]); //NEW

		// Configura o openGL
		initOpenGL();

		// Inicializa FingertipPoseEstimation
		if (false == fingertipPoseEstimation.initialize(calibrationFileName, Size(capture.getFrameWidth(), capture.getFrameHeigth())))
			return 0;

		// Start main Glut loop
		glutMainLoop();

	}
	catch (exception &e) {
		cout << e.what() << endl;
		system("pause");
	}

	return 0;

}

void initOpenGL() {

	// Modo de exibis�o
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	// Posi��o da janela
	glutInitWindowPosition(100, 100);
	// Criando janela Glut - Tamanho
	glutInitWindowSize(capture.getFrameWidth(), capture.getFrameHeigth());
	// Criando janela Glut - Titulo
	glutCreateWindow("Painel de Realidade Aumentada");


	// Ativando o z-buufer
	glEnable(GL_DEPTH_TEST);

	// Propriedade para a normaliza��o de vetores
	glEnable(GL_NORMALIZE);

	// Ativa a ilumina��o padr�o
	glEnable(GL_LIGHTING);

	// Ilumina��o...-------------------------------------
	GLfloat light_position[] = { 100.0, 500, 200, 1.0 };
	GLfloat white_light[] = { 1.0, 1.0, 1.0, 0.8 };
	GLfloat lmodel_ambient[] = { 0.9, 0.9, 0.9, 0.5 };

	glClearColor(0.0, 0.0, 0.0, 0.0);
	glShadeModel(GL_SMOOTH);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, white_light);
	glLightfv(GL_LIGHT0, GL_SPECULAR, white_light);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
	// ----------------------------------------------------

	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_TEXTURE_2D);
	glCullFace(GL_BACK);
	glFrontFace(GL_CW);

	// Init Earth Texture
	glGenTextures(1, &gnEarthTexID);
	glBindTexture(GL_TEXTURE_2D, gnEarthTexID);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 256, 256, 0, GL_RGB, GL_UNSIGNED_BYTE, 0);
	
	Mat mat = imread(earthTextureFileName);
	cvtColor(mat, mat, CV_RGB2BGR); // convert image R and B channel
	flip(mat, mat, 0); /*####################################################* CUIDADO */
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, mat.cols, mat.rows, GL_RGB, GL_UNSIGNED_BYTE, mat.data);

	// Fun��es Callback
	glutDisplayFunc(display);
	glutIdleFunc(idle);

}


void display()
{
	// Desenha "as coisas" na m�o 
	// display info
	fingertipPoseEstimation.OnDisplay();

	// check if there have been any openGL problems
	GLenum errCode = glGetError();
	if (errCode != GL_NO_ERROR) {
		const GLubyte *errString = gluErrorString(errCode);
		fprintf(stderr, "OpenGL error: %s\n", errString);
	}

	// clear the buffers of the last frame
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

	// set the drawing region of the window
	glViewport(0, 0, capture.getFrameWidth(), capture.getFrameHeigth());

	// set viewing frustrum to match camera FOV (ref: ARTag's 3d_augmentations.cpp)
	fingertipPoseEstimation.setOpenGLFrustrum();

	// set modelview matrix of the camera
	fingertipPoseEstimation.setOpenGLModelView();

	// render virtual objects
	GLfloat colorRed[3] = { 1.0, 0.0, 0.0 };
	GLfloat colorGreen[3] = { 0.0, 1.0, 0.0 };
	GLfloat colorBlue[3] = { 0.0, 0.0, 1.0 };
	GLfloat colorYellow[3] = { 1.0, 1.0, 0.0 };
	GLfloat colorDarkYellow[3] = { 0.6, 0.6, 0.0 };
	GLfloat colorWhite[3] = { 1.0, 1.0, 1.0 };
	GLfloat colorGray[3] = { 0.5, 0.5, 0.5 };


	if (fingertipPoseEstimation.validPose()) {
		glPushMatrix();

		if (gnModel == MODEL_EARTH) {
		
			/*Mat _image;
			video.read(_image);
			resize(_image, _image, Size(256, 256));

			imshow("Video", _image);
			//_image = imread(earthTextureFileName);

			cvtColor(_image, _image, CV_RGB2BGR);
			flip(_image, _image, 0);


			glGenTextures(1, &gnEarthTexID);
			glBindTexture(GL_TEXTURE_2D, gnEarthTexID);

			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 256, 256, 0, GL_RGB, GL_UNSIGNED_BYTE, 0);
			glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, _image.cols, _image.rows, GL_RGB, GL_UNSIGNED_BYTE, _image.data);*/
			
			
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, gnEarthTexID);
			GLUquadricObj *quadObj;
			quadObj = gluNewQuadric();
			gluQuadricDrawStyle(quadObj, GLU_FILL);
			gluQuadricTexture(quadObj, GL_TRUE);
			glPushMatrix();
			glTranslatef(0, 0, 50);
			gluSphere(quadObj, 50, 32, 32);
			glPopMatrix();
			glDisable(GL_TEXTURE_2D);
		}
		else if (gnModel == MODEL_COORDINATE_AXES) {
			glDisable(GL_LIGHTING);
			glLineWidth(3);
			glBegin(GL_LINES);
			glMaterialfv(GL_FRONT, GL_AMBIENT, colorRed);
			glColor3fv(colorRed);
			glVertex3f(0, 0, 0);
			glVertex3f(100, 0, 0);
			glMaterialfv(GL_FRONT, GL_AMBIENT, colorGreen);
			glColor3fv(colorGreen);
			glVertex3f(0, 0, 0);
			glVertex3f(0, 100, 0);
			glMaterialfv(GL_FRONT, GL_AMBIENT, colorBlue);
			glColor3fv(colorBlue);
			glVertex3f(0, 0, 0);
			glVertex3f(0, 0, 100);
			glEnd();
			glPushMatrix();
			glColor3fv(colorRed);
			glTranslatef(100, 0, 0);
			glRotatef(90, 0, 1, 0);
			glutSolidCone(5, 10, 16, 16);
			glPopMatrix();
			glPushMatrix();
			glColor3fv(colorGreen);
			glTranslatef(0, 100, 0);
			glRotatef(-90, 1, 0, 0);
			glutSolidCone(5, 10, 16, 16);
			glPopMatrix();
			glPushMatrix();
			glColor3fv(colorBlue);
			glTranslatef(0, 0, 100);
			glRotatef(90, 0, 0, 1);
			glutSolidCone(5, 10, 16, 16);
			glPopMatrix();
			glEnable(GL_LIGHTING);
		}

		glPopMatrix();

	}

	glutSwapBuffers();
}

void idle()
{

	//try {

		// Pego o proximo frame
		Mat &frame = capture.nextFrame();

		// Se o frame for vazio
		if (frame.empty()) return;

		// Passa o frame para fingertipPoseEstimation
		fingertipPoseEstimation.OnCapture(frame);

		// Realiza o processamento da imagem
		fingertipPoseEstimation.OnProcess();

		// Desenha objetos
		glutPostRedisplay();

	/*}
	catch (ArException &e) {
		cout << e.what() << endl;
	}*/
}