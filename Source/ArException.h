#pragma once

#include <exception>
#include <iostream>

using namespace std;

class ArException : public exception
{
public:
	ArException(string classEx, string methodEx, exception ex);

	string what();

private:
	string _classEx;
	string _methodEx;
};

