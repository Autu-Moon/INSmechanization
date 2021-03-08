//
// Created by sheldon on 20-11-3.
//

#ifndef INSMECH_IMUDATA_H
#define INSMECH_IMUDATA_H

#include "constant.h"
#include <fstream>
#include <iostream>
#include<iomanip>
#include <vector>
#include <Eigen/Dense>
using namespace std;
struct IMUinfo {
	double              epoch;               //��Ԫ��Ϣ          unit:s
	Eigen::Vector3d     gyro;                //��������Ľ�����   unit:rad
	Eigen::Vector3d     accm;                //�ӱ�������ٶȱ仯 unit:m/s
};
class IMUreader
{
public:
	//����imu��������� 
	//���꣺	ǰ �� ��
	//��ʽ��	����
	//��λ��	rad  m/s
	std::vector<IMUinfo>    imuvec;				
	bool                getExamplePureData(const char* filename);
	bool				decodeExamplePureData(const char* infilename, const char* outfilename);
	bool				getLideA15Data(const char* filename);
	double              R8decode(const char* buff);
};
#endif //INSMECH_IMUDATA_H
