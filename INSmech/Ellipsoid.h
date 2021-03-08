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
	double          a;                                      //长半轴
	double          b;                                      //短半轴
	double          f;                                      //扁率
	double          esq;									//第一偏心率平方
	const double    GM = 3.986005e14;
	const double    g_a = 9.7803267715;						//赤道处重力 unit:m/s^2
	const double    g_b = 9.8321863685;						//极点处重力 unit:m/s^2
public:
	//构造函数来获得椭球参数
	Ellipsoid(double a1, double f1)
	{
		a = a1;
		f = f1;
		esq = 2 * f - f * f;
		b = (1 - f)*a;
	}

	Ellipsoid()
	{
		//默认WGS84参数
		a = 6378137.0;
		f = 1 / 298.257223563;                                //0.003352810664747
		esq = 2 * f - f * f;
		b = (1 - f)*a;
	}
	double computeR_M(const double &latitude);              //子午圈半径
	double computeR_N(const double &latitude);              //卯酉圈半径
	double computeNormalGravity(const double &latitude      //正常重力计算
		, const double &h);
};
#endif //INSMECH_ELLIPSOID_H
