#pragma once
//
// Created by sheldon on 20-11-4.
//

#ifndef INSMECH_ELLIPSOID_H
#define INSMECH_ELLIPSOID_H

#include "constant.h"
#include <cmath>
class Ellipsoid
{
private:
	double          a;                                      //������
	double          b;                                      //�̰���
	double          f;                                      //����
	double          esq;									//��һƫ����ƽ��
	const double    GM = 3.986005e14;
	const double    g_a = 9.7803267715;						//��������� unit:m/s^2
	const double    g_b = 9.8321863685;						//���㴦���� unit:m/s^2
public:
	//���캯��������������
	Ellipsoid(double a1, double f1)
	{
		a = a1;
		f = f1;
		esq = 2 * f - f * f;
		b = (1 - f)*a;
	}

	Ellipsoid()
	{
		//Ĭ��WGS84����
		a = 6378137.0;
		f = 1 / 298.257223563;                                //0.003352810664747
		esq = 2 * f - f * f;
		b = (1 - f)*a;
	}
	double computeR_M(const double &latitude);              //����Ȧ�뾶
	double computeR_N(const double &latitude);              //î��Ȧ�뾶
	double computeNormalGravity(const double &latitude      //������������
		, const double &h);
};
#endif //INSMECH_ELLIPSOID_H
