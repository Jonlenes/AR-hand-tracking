#include "SavePrint.h"

SavePrint* SavePrint::instance = 0;

SavePrint::SavePrint()
{

}

SavePrint* SavePrint::getInstance()
{
	if (!instance)
		instance = new SavePrint();
	return instance;
}

void SavePrint::save(string key, Mat image, int a)
{
	//if (mKeys.find(key) == mKeys.end())
	//	mKeys[key] = -1;
	//mKeys[key]++;
	//imwrite(path + key + "_" + to_string(mKeys[key]) + ".png", image);
}