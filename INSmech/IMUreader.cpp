//
// Created by sheldon on 20-11-3.
//
#define _CRT_SECURE_NO_WARNINGS
#include <cstring>
#include "IMUreader.h"
bool IMUreader::getExamplePureData(const char *filename)
{
	//格式： 周秒  陀螺XYZ(rad)  加表XYZ(m/s)
	const int MAXIMUMESGLEN = 100;
	char buff[MAXIMUMESGLEN];
	FILE *fin = fopen(filename, "r+b");
	while (!feof(fin)) {
		if (fread(buff, 8, 7, fin) < 7)
			break;
		IMUinfo imu;
		for (int i = 0; i < 7; ++i) {
			if (i == 0) {
				imu.epoch = R8decode(buff + i * 8);
			}
			else if (i > 0 && i < 4) {
				imu.gyro[i - 1] = R8decode(buff + i * 8);
			}
			else {
				imu.accm[i - 4] = R8decode(buff + i * 8);
			}
		}
		imuvec.push_back(imu);
		memset(buff, 0, MAXIMUMESGLEN);
	}
	return true;
}

bool IMUreader::getLideA15Data(const char* filename)
{
	//格式：周秒 陀螺XYZ(rad) 加表XYZ(m/s)
	const int MAXIMUMESGLEN = 100;
	char buff[MAXIMUMESGLEN];
	FILE *fin = fopen(filename, "r+b");
	while (!feof(fin)) {
		if (fread(buff, 8, 7, fin) < 7)
			break;
		IMUinfo imu;
		for (int i = 0; i < 7; ++i) {
			if (i == 0) {
				imu.epoch = R8decode(buff + i * 8);
			}
			else if (i > 0 && i < 4) {
				imu.gyro[i - 1] = (R8decode(buff + i * 8));
			}
			else {
				imu.accm[i - 4] = R8decode(buff + i * 8);
			}
		}
		imuvec.push_back(imu);
		memset(buff, 0, MAXIMUMESGLEN);
	}
	return true;
}
bool IMUreader::decodeExamplePureData(const char* infilename, const char* outfilename)
{
	const int MAXIMUMESGLEN = 100;
	char buff[MAXIMUMESGLEN];
	FILE *fin = fopen(infilename, "r+b"); 
	std::ofstream fout;
	fout.open(outfilename, ios::out | ios::app);
	fout.setf(ios::fixed, ios::floatfield);
	fout << std::right;

	
	while (!feof(fin)) {
		if (fread(buff, 8, 7, fin) < 7)
			break;
		IMUinfo imu;
		for (int i = 0; i < 7; ++i) {
			if (i == 0) {
				imu.epoch = R8decode(buff + i * 8);
			}
			else if (i > 0 && i < 4) {
				imu.gyro[i - 1] = (R8decode(buff + i * 8));
			}
			else {
				imu.accm[i - 4] = R8decode(buff + i * 8);
			}
			
		}
		fout << setprecision(15) << " " << imu.epoch << " " << imu.gyro[0] << " " << imu.gyro[1] << " " << imu.gyro[2]
			<< " " << imu.accm[0] << " " << imu.accm[1] << " " << imu.accm[2] << endl;
		memset(buff, 0, MAXIMUMESGLEN);
	}
	return true;
}
double IMUreader::R8decode(const char *buff) {
	double r;
	memcpy(&r, buff, 8);
	return r;
}