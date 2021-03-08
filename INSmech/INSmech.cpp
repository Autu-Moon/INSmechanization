//
// Created by sheldon on 20-11-4.
//

#include "INSmech.h"
bool INSmech::Solve(const std::vector<IMUinfo> &imuvec, const char *outfile)
{
	if (iniInfo.Pos.norm() == 0 && iniInfo.Att.norm() == 0) {                         //检查是否有初始对准信息
		std::cerr << "No Initial PVA data!" << std::endl;
		return false;
	}
	NavInfo epochInfo;
	posDeque.push_back(iniInfo.Pos);
	velDeque.push_back(iniInfo.Vel);
	attDeque.push_back(iniInfo.Att);
	cout.setf(ios::fixed, ios::floatfield);
	cout << setprecision(8);
	for (int i = 1; i < imuvec.size(); i++) {                                       //逐历元迭代
		if (imuvec[i].epoch < iniInfo.epoch
			&&abs(imuvec[i].epoch - iniInfo.epoch) > delta_t / 5) {				//在初始化历元前的不进行解算
			continue;
		}
		if (imuDeque.size() < 1) {
			imuDeque.push_back(imuvec[i]);											//第一个历元不进行解算
			continue;
		}
		else {
			updateData(imuvec[i], imuDeque);                                        //数据更新
		}
		updateAtt();                                                                //姿态更新
		updateVel();                                                                //速度更新
		updatePos();                                                                //位置更新
		epochInfo.epoch = imuDeque.back().epoch;
		epochInfo.Pos = posDeque.back();
		epochInfo.Vel = velDeque.back();
		epochInfo.Att = attDeque.back();
		writeFile(outfile, epochInfo);
		/*cout.setf(ios::fixed, ios::floatfield);
		cout << std::right;

		cout << setprecision(3) << setw(11) << epochPva.epoch << " Pos:" << setprecision(12) << setw(18) << Rad2Deg(epochPva.Pos[0]) << setw(18) << Rad2Deg(epochPva.Pos[1]) << setw(18) << epochPva.Pos[2]
			<< setw(5) << " Vel:" << epochPva.Vel.transpose()
			<< setw(5) << " Att:" << 180 / M_PI * epochPva.Att.transpose() << endl;*/
	}
	return true;
}
void INSmech::updateVel()
{
	Eigen::Vector3d     Wn_ie = computeWn_ie(posDeque.back()(0));
	Eigen::Vector3d     Wn_en = computeWn_en(posDeque.back()(0), posDeque.back()(2), velDeque.back()(0), velDeque.back()(1));
	Eigen::Matrix3d     Cnk1_bk1 = EulerToDCM(attDeque.back());
	Eigen::Matrix3d     I = Matrix3d::Identity();
	//哥氏积分项
	Eigen::Vector3d     gn_l(0.0, 0.0, ellipsoid.computeNormalGravity(posDeque.back()(0), posDeque.back()(2)));
	Eigen::Vector3d     Vn_g = delta_t * (gn_l - (2 * Wn_ie + Wn_en).cross(velDeque.back()));                  //哥氏积分项
	//比力积分项
	Eigen::Vector3d     Vbk1_fk = imuDeque[1].accm + 0.5*imuDeque[1].gyro.cross(imuDeque[1].accm) +
		(1.0 / 12.)*(imuDeque[0].gyro.cross(imuDeque[1].accm) + imuDeque[0].accm.cross(imuDeque[1].gyro));
	Eigen::Vector3d     Zeta_k1k = delta_t * (Wn_en + Wn_ie);
	Eigen::Vector3d     Vn_fk = (I - 0.5*Vec3dToSSymmeticMatrix(Zeta_k1k))*Cnk1_bk1*Vbk1_fk;
	//速度更新
	Eigen::Vector3d     newVel = velDeque.back() + Vn_fk + Vn_g;
	updateData(newVel, velDeque);
}

void INSmech::updateAtt()
{
	Eigen::Vector3d     Wn_ie = computeWn_ie(posDeque.back()(0));
	Eigen::Vector3d     Wn_en = computeWn_en(posDeque.back()(0), posDeque.back()(2), velDeque.back()(0), velDeque.back()(1));
	//b系更新
	Eigen::Vector3d     Phi_k = imuDeque[1].gyro + (1.0 / 12.0)*imuDeque[0].gyro.cross(imuDeque[1].gyro);
	double              phinorm05 = 0.5*Phi_k.norm();
	Eigen::Quaterniond  qbk1_bk(cos(phinorm05),
		sin(phinorm05)*Phi_k[0] / (phinorm05 * 2),
		sin(phinorm05)*Phi_k[1] / (phinorm05 * 2),
		sin(phinorm05)*Phi_k[2] / (phinorm05 * 2));
	//n系更新
	Eigen::Vector3d     Zeta_kk1 = delta_t * (Wn_en + Wn_ie);
	double              Zeta05 = 0.5*Zeta_kk1.norm();
	Eigen::Quaterniond  qnk_nk1(cos(Zeta05),
		-sin(Zeta05)*Zeta_kk1[0] / (Zeta05 * 2),
		-sin(Zeta05)*Zeta_kk1[1] / (Zeta05 * 2),
		-sin(Zeta05)*Zeta_kk1[2] / (Zeta05 * 2));
	//姿态更新
	Eigen::Quaterniond  qnk_bk = qnk_nk1 * EulerToQuaerntion(attDeque.back())*qbk1_bk;
	qnk_bk.normalize();										//归一化
	Eigen::Vector3d		newAtt = DCMToEuler210(qnk_bk.matrix());
	updateData(newAtt, attDeque);
}

void INSmech::updatePos()
{
	//高程
	double              hk = posDeque.back()(2) - 1.0 / 2. * (velDeque[0](2) + velDeque[1](2))*delta_t;
	//纬度
	double              hk_ave = 1.0 / 2. * (hk + posDeque.back()(2));
	double              phik = posDeque.back()(0) + 1.0 / 2. * (velDeque[0](0) + velDeque[1](0))
		/ (ellipsoid.computeR_M(posDeque.back()(0)) + hk_ave)*delta_t;
	//经度
	double              phik_ave = 1.0 / 2. * (phik + posDeque.back()(0));
	double              gamak = posDeque.back()(1) + 1.0 / 2. * (velDeque[0](1) + velDeque[1](1))
		/ ((ellipsoid.computeR_N(phik_ave) + hk_ave)*cos(phik_ave))*delta_t;
	Eigen::Vector3d     newPos(phik, gamak, hk);
	updateData(newPos, posDeque);
}

Eigen::Vector3d INSmech::computeWn_ie(const double &latitude)
{
	Eigen::Vector3d Wn_ie(Omega_e*cos(latitude), 0, -Omega_e * sin(latitude));
	return Wn_ie;
}

Eigen::Vector3d INSmech::computeWn_en(const double &latitude, const double &h, const double &V_N, const double &V_E)
{
	double R_N = ellipsoid.computeR_N(latitude);
	double R_M = ellipsoid.computeR_M(latitude);
	Eigen::Vector3d Wn_en(V_E / (R_N + h), -V_N / (R_M + h), -V_E * tan(latitude) / (R_N + h));
	return Wn_en;
}

Eigen::Quaterniond INSmech::EulerToQuaerntion(const Eigen::Vector3d &euler)
{
	double              roll = euler(0);
	double              pitch = euler(1);
	double              yaw = euler(2);
	Eigen::Quaterniond  quaterniond(
		cos(roll / 2.)*cos(pitch / 2.)*cos(yaw / 2.) + sin(roll / 2.)*sin(pitch / 2.)*sin(yaw / 2.),
		sin(roll / 2.)*cos(pitch / 2.)*cos(yaw / 2.) - cos(roll / 2.)*sin(pitch / 2.)*sin(yaw / 2.),
		cos(roll / 2.)*sin(pitch / 2.)*cos(yaw / 2.) + sin(roll / 2.)*cos(pitch / 2.)*sin(yaw / 2.),
		cos(roll / 2.)*cos(pitch / 2.)*sin(yaw / 2.) - sin(roll / 2.)*sin(pitch / 2.)*cos(yaw / 2.));
	return  quaterniond;
}

Eigen::Matrix3d INSmech::Vec3dToSSymmeticMatrix(const Eigen::Vector3d &v3d)
{
	Matrix3d matrix3d = Matrix3d::Zero();                 //矩阵初始化
	matrix3d << 0, -v3d[2], v3d[1],
		v3d[2], 0, -v3d[0],
		-v3d[1], v3d[0], 0;
	return matrix3d;
}

Eigen::Matrix3d	INSmech::EulerToDCM(const Eigen::Vector3d& euler)
{
	double              roll = euler(0);
	double              pitch = euler(1);
	double              yaw = euler(2);
	Matrix3d			Cb_n = Matrix3d::Zero();
	Cb_n << cos(pitch)*cos(yaw), -cos(roll)*sin(yaw) + sin(roll)*sin(pitch)*cos(yaw), sin(roll)*sin(yaw) + cos(roll)*sin(pitch)*cos(yaw),
		cos(pitch)*sin(yaw), cos(roll)*cos(yaw) + sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw) + cos(roll)*sin(pitch)*sin(yaw),
		-sin(pitch), sin(roll)*cos(pitch), cos(roll)*cos(pitch);
	return Cb_n;
}

Eigen::Vector3d INSmech::DCMToEuler210(const Eigen::Matrix3d &DCM)
{
	double roll = atan2(DCM(2, 1), DCM(2, 2));
	double pitch = atan(-DCM(2, 0) / sqrt(DCM(2, 1)*DCM(2, 1) + DCM(2, 2)*DCM(2, 2)));
	double yaw = atan2(DCM(1, 0), DCM(0, 0));
	Eigen::Vector3d eulerAngle(roll, pitch, yaw);
	return eulerAngle;
}

void INSmech::updateData(const IMUinfo &imu, std::deque<IMUinfo> &deque)
{
	if (deque.size() < 2)                      //若deque中数据不足两个则不删除第一个
		deque.push_back(imu);
	else {                                   //正常情况 删第一个 存新的
		deque.pop_front();
		deque.push_back(imu);
	}
}

void INSmech::updateData(const Vector3d &vec3d, std::deque<Vector3d> &deque)
{
	if (deque.size() < 2)                   //若deque中数据不足两个则不删除第一个
		deque.push_back(vec3d);
	else {                                //正常情况 删第一个 存新的
		deque.pop_front();
		deque.push_back(vec3d);
	}
}

void INSmech::writeFile(const char* filename, const NavInfo& pva)
{
	std::ofstream fout;
	fout.open(filename, ios::out | ios::app);
	fout.setf(ios::fixed, ios::floatfield);
	fout << std::right;

	fout << setprecision(3) << setw(11) << pva.epoch  << setprecision(12) << setw(18) << Rad2Deg(pva.Pos[0]) << setw(18) << Rad2Deg(pva.Pos[1]) << setw(23) << pva.Pos[2]
		<< setw(5)  << pva.Vel.transpose()
		<< setw(5)  << 180 / M_PI * pva.Att.transpose() << endl;
}

//template <typename T>
//void INSmech::updateData(const T &imu, std::deque<T> &deque) {
//    if (deque.size() < 2)                      //若deque中数据不足两个则不删除第一个
//        deque.push_back(imu);
//    else {                                   //正常情况 删第一个 存新的
//        deque.pop_front();
//        deque.push_back(imu);
//    }
//}