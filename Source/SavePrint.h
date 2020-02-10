#pragma once

#include <iostream>
#include <map>
#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

const string path = "C:/Users/asus/Dropbox/PrintsTcc/";

class SavePrint
{
public:

	static SavePrint* getInstance();
	void save(string key, Mat image, int a);

private:
	SavePrint();

	static SavePrint *instance;
	map<string, long> mKeys;
};

