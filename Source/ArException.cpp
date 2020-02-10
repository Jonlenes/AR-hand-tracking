#include "ArException.h"


ArException::ArException(string classEx, string methodEx, exception ex)
	: _classEx(classEx), _methodEx(methodEx), exception(ex)
{

}

string ArException::what()
{
	return _classEx + " - " + _methodEx + ": " + string(exception::what());
}