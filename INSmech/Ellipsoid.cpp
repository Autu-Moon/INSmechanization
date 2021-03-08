//
// Created by sheldon on 20-11-4.
//
#include "Ellipsoid.h"
double Ellipsoid::computeR_M(const double &latitude) {
	return (a*(1 - esq) /
		pow(sqrt(1.0 - esq * sin(latitude)*sin(latitude)), 3));
}

double Ellipsoid::computeR_N(const double &latitude) {
	return (a / sqrt(1.0 - esq * sin(latitude)*sin(latitude)));
}

double Ellipsoid::computeNormalGravity(const double &latitude, const double &h) {
	double normalG_0 = (a*g_a*cos(latitude)*cos(latitude) + b * g_b*sin(latitude)*sin(latitude)) /
		sqrt(a*a*cos(latitude)*cos(latitude) + b * b*sin(latitude)*sin(latitude));           //高程为0的正常重力
	double m = Omega_e * Omega_e*a*a*b / GM;
	double normalG_h = normalG_0 * (1.0 - 2.0 / a * (1.0 + f + m - 2.0 * f*sin(latitude)*sin(latitude))*h
		+ 3.0 * h*h / (a*a));
	return normalG_h;
}

