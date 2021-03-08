#include <iostream>
#include <Eigen/Dense>
#include "IMUreader.h"
#include "INSmech.h"

using namespace Eigen;
//�ߵ���е����ʵ��
//����ϵ��ǰ �� �� F R D
//����ϵ���� �� �� N E U
int main() {
	//����A15����ϳ�ʼ��̬
	Eigen::Vector3d initialPos_Loose(Deg2Rad(30.4444577064)								//B L H unit: rad rad m
		, Deg2Rad(114.4718750262), 20.814);
	Eigen::Vector3d initialVec_Loose(-7.430, -0.003, 0.033);								//unit:m/s
	Eigen::Vector3d initialAtt_Loose(Deg2Rad(0.78590517),								//roll pitch yaw unit:rad
		Deg2Rad(-1.73830101), Deg2Rad(181.19926685));
	double			initialepoch_Loose = 456371.000;
	//����A15���ߵ���ʼ��̬
	//Eigen::Vector3d initialPos_pureINS(Deg2Rad(30.4445199466)								//B L H unit: rad rad m
	//	, Deg2Rad(114.4718777663), 21.016);
	//Eigen::Vector3d initialVec_pureINS(-6.360, -0.040, 0.049);								//unit:m/s
	//Eigen::Vector3d initialAtt_pureINS(Deg2Rad(-0.00137516),								//roll pitch yaw unit:rad
	//	Deg2Rad(-0.43093389),Deg2Rad(180.38599896)-2*M_PI);
	//double			initialepoch_pureINS = 456370.009;
	
	IMUreader imuData;                                                
	INSmech insMech(initialepoch_Loose, initialPos_Loose, initialVec_Loose, initialAtt_Loose);				//��ʼ��
	char file[] = "A15_imu.imd";													//��ȡ����
	char outfile[] = "A15_loose_av.txt";
	imuData.getExamplePureData(file);
	insMech.Solve(imuData.imuvec, outfile);
	system("pause");
	return 0;
}