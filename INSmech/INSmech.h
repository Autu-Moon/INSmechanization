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
struct NavInfo                                                          //�˶�P V A ��Ϣ
{
	double					epoch;
	Eigen::Vector3d         Att;                                    //roll pitch yaw
	Eigen::Vector3d         Vel;                                    //ǰ �� ��
	Eigen::Vector3d         Pos;                                    //B  L  H
	NavInfo() {}

	NavInfo(double t, const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& a) :
		epoch(t), Pos(p), Vel(v), Att(a) {}
};
class INSmech
{
public:
	Ellipsoid                           ellipsoid;                                                      //WGS-84����
	double                              delta_t = 0.005;                                                 //������� unit��s
	//��ʼ��׼
	NavInfo                             iniInfo;														//yaw pitch roll(2,1,0)//ǰ �� ��//B L H
	std::deque<IMUinfo>                     imuDeque;                                                       //ǰһ��Ԫ����һ��Ԫ��imu����
	std::deque<Eigen::Vector3d>         posDeque;                                                       //����ǰһ��Ԫ����һ��Ԫ��λ��
	std::deque<Eigen::Vector3d>         velDeque;                                                       //�ٶ�
	std::deque<Eigen::Vector3d>			attDeque;                                                       //��̬
	//std::vector<PVA>                    pvavec;                                                       //����������Ԫ�˶���Ϣ
	bool                                Solve(const std::vector<IMUinfo>& imuvec, const char *outfile);		//�ߵ������ܺ���
	void                                updateVel();                                                    //�ٶȸ���
	void                                updateAtt();                                                    //��̬����
	void                                updatePos();                                                    //λ�ø���
	//template <typename T> void          updateData(const T &element,std::deque<T> &deque);
	void                                updateData(const IMUinfo &imu, std::deque<IMUinfo> &deque);             //��IMUdeque����
	void                                updateData(const Eigen::Vector3d &vec3d,                        //��V3ddeque����
													std::deque<Eigen::Vector3d> &deque);
	void								writeFile(const char* filename, const NavInfo& pva);
	Eigen::Vector3d                     computeWn_ie(const double& latitude);                           //����Wn_ie
	Eigen::Vector3d                     computeWn_en(const double &latitude, const double &h,           //����Wn_en
														const double &V_N, const double &V_E);

	//ת�����ߺ���
	Eigen::Quaterniond                  EulerToQuaerntion(const  Eigen::Vector3d& euler);				//ŷ����ת��Ԫ��
	Eigen::Matrix3d                     Vec3dToSSymmeticMatrix(const  Eigen::Vector3d& v3d);			//��ά����ת���Գƾ���
	Eigen::Matrix3d						EulerToDCM(const  Eigen::Vector3d& euler);						//ŷ����תDCM
	Eigen::Vector3d						DCMToEuler210(const  Eigen::Matrix3d& DCM);						//DCMתŷ����	
	INSmech() {}
	INSmech(double t, const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& a) :
		iniInfo(t, p, v, a) {}
};

#endif //INSMECH_INSMECH_H
