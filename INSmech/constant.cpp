#include"constant.h"

double Deg2Rad(const double& deg)						//角度转弧度
{
	double pi = 3.141592653589793;
	return(deg / 180.0*pi);
}
double Rad2Deg(const double& rad)						//弧度转角度
{
	double pi = 3.141592653589793;
	return(rad / pi * 180.0);
}