#include"constant.h"

double Deg2Rad(const double& deg)						//�Ƕ�ת����
{
	double pi = 3.141592653589793;
	return(deg / 180.0*pi);
}
double Rad2Deg(const double& rad)						//����ת�Ƕ�
{
	double pi = 3.141592653589793;
	return(rad / pi * 180.0);
}