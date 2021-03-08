#pragma once
//
// Created by sheldon on 20-11-3.
//

#ifndef INSMECH_INSMECH_H
#define INSMECH_INSMECH_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include<iostream>
#include "IMUreader.h"
#include "constant.h"
#include "Ellipsoid.h"
#include <deque>
#include <vector>
#include<iomanip>
#include<cmath>
#include<fstream>
using namespace Eigen;
using namespace std;
using std::deque;
using std::vector;
struct NavInfo                                                          //运动P V A 信息
{
	double					epoch;
	Eigen::Vector3d         Att;                                    //roll pitch yaw
	Eigen::Vector3d         Vel;                                    //前 右 下
	Eigen::Vector3d         Pos;                                    //B  L  H
	NavInfo() {}

	NavInfo(double t, const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& a) :
		epoch(t), Pos(p), Vel(v), Att(a) {}
};
class INSmech
{
public:
	Ellipsoid                           ellipsoid;                                                      //WGS-84椭球
	double                              delta_t = 0.005;                                                 //采样间隔 unit：s
	//初始对准
	NavInfo                             iniInfo;														//yaw pitch roll(2,1,0)//前 右 下//B L H
	std::deque<IMUinfo>                     imuDeque;                                                       //前一历元和现一历元的imu数据
	std::deque<Eigen::Vector3d>         posDeque;                                                       //储存前一历元和现一历元的位置
	std::deque<Eigen::Vector3d>         velDeque;                                                       //速度
	std::deque<Eigen::Vector3d>			attDeque;                                                       //姿态
	//std::vector<PVA>                    pvavec;                                                       //储存所有历元运动信息
	bool                                Solve(const std::vector<IMUinfo>& imuvec, const char *outfile);		//惯导解算总函数
	void                                updateVel();                                                    //速度更新
	void                                updateAtt();                                                    //姿态更新
	void                                updatePos();                                                    //位置更新
	//template <typename T> void          updateData(const T &element,std::deque<T> &deque);
	void                                updateData(const IMUinfo &imu, std::deque<IMUinfo> &deque);             //对IMUdeque更新
	void                                updateData(const Eigen::Vector3d &vec3d,                        //对V3ddeque更新
													std::deque<Eigen::Vector3d> &deque);
	void								writeFile(const char* filename, const NavInfo& pva);
	Eigen::Vector3d                     computeWn_ie(const double& latitude);                           //计算Wn_ie
	Eigen::Vector3d                     computeWn_en(const double &latitude, const double &h,           //计算Wn_en
														const double &V_N, const double &V_E);

	//转换工具函数
	Eigen::Quaterniond                  EulerToQuaerntion(const  Eigen::Vector3d& euler);				//欧拉角转四元数
	Eigen::Matrix3d                     Vec3dToSSymmeticMatrix(const  Eigen::Vector3d& v3d);			//三维向量转反对称矩阵
	Eigen::Matrix3d						EulerToDCM(const  Eigen::Vector3d& euler);						//欧拉角转DCM
	Eigen::Vector3d						DCMToEuler210(const  Eigen::Matrix3d& DCM);						//DCM转欧拉角	
	INSmech() {}
	INSmech(double t, const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& a) :
		iniInfo(t, p, v, a) {}
};

#endif //INSMECH_INSMECH_H
