#pragma once
class Util
{
public:
	Util();


	static long mod(long v, long n)
	{
		return ((((v / n) + 1)*n + v) % n);
	}
};

