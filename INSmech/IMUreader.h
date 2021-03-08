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
	double              epoch;               //历元信息          unit:s
	Eigen::Vector3d     gyro;                //陀螺输出的角增量   unit:rad
	Eigen::Vector3d     accm;                //加表输出的速度变化 unit:m/s
};
class IMUreader
{
public:
	//储存imu输出的数据 
	//坐标：	前 右 下
	//形式：	增量
	//单位：	rad  m/s
	std::vector<IMUinfo>    imuvec;				
	bool                getExamplePureData(const char* filename);
	bool				decodeExamplePureData(const char* infilename, const char* outfilename);
	bool				getLideA15Data(const char* filename);
	double              R8decode(const char* buff);
};
#endif //INSMECH_IMUDATA_H
