#include"Kinematics.h"
Kinematics* Kinematics::CreateKinematics(int model, ST_DH dh)
{
	if (model == 0)
		return new Kinematics_E(dh);
	else
		return new Kinematics_S(dh);
}
Kinematics::Kinematics(int nModel ,ST_DH dh)
{
	return;
}
Kinematics::~Kinematics()
{
	return;
}

Kinematics_E::Kinematics_E(ST_DH dh) : Kinematics(0, dh)
{
	m_DH = dh;
	std::vector<double> linkParam;
	m_nRobotType = 0;

	linkParam.insert(linkParam.end(), { 0,       m_DH.d1,     0,       PI / 2,    0,   0 }); m_Link.push_back(Link(linkParam)); linkParam.clear();
	linkParam.insert(linkParam.end(), { PI / 2,  0,           m_DH.a2, PI,        0,   PI / 2 }); m_Link.push_back(Link(linkParam)); linkParam.clear();
	linkParam.insert(linkParam.end(), { PI / 2,  0,           0,       PI / 2,    0,   PI / 2 }); m_Link.push_back(Link(linkParam)); linkParam.clear();
	linkParam.insert(linkParam.end(), { 0,       m_DH.d4,     0,      -PI / 2,    0,   0 }); m_Link.push_back(Link(linkParam)); linkParam.clear();
	linkParam.insert(linkParam.end(), { 0,       0,			  0,       PI / 2,    0,   0 }); m_Link.push_back(Link(linkParam)); linkParam.clear();
	linkParam.insert(linkParam.end(), { 0,       m_DH.d6,     0,       0,         0,   0 }); m_Link.push_back(Link(linkParam)); linkParam.clear();
	//默认RLR
	sType.GCS = EnumArmType_R;
	sType.GCE = EnumArmType_L;
	sType.GCW = EnumArmType_R;
}

Kinematics_S::Kinematics_S(ST_DH dh) : Kinematics(1, dh)
{
	m_DH = dh;
	std::vector<double> linkParam;
	m_nRobotType = 1;
	dh.d4 = dh.d2 - dh.d3 + dh.d4;
	linkParam.insert(linkParam.end(), { 0,       dh.d1,     0,       PI / 2,		0,   0 }); m_Link.push_back(Link(linkParam)); linkParam.clear();
	linkParam.insert(linkParam.end(), { 0,		 0,           dh.a2, 0,			0,   0 }); m_Link.push_back(Link(linkParam)); linkParam.clear();
	linkParam.insert(linkParam.end(), { 0,		 0,           dh.a3, 0,			0,   0 }); m_Link.push_back(Link(linkParam)); linkParam.clear();
	linkParam.insert(linkParam.end(), { 0,       dh.d4,     0,       PI / 2,		0,   0 }); m_Link.push_back(Link(linkParam)); linkParam.clear();
	linkParam.insert(linkParam.end(), { 0,       dh.d5,     0,       -PI / 2,		0,   0 }); m_Link.push_back(Link(linkParam)); linkParam.clear();
	linkParam.insert(linkParam.end(), { 0,       dh.d6,     0,       0,			0,   0 }); m_Link.push_back(Link(linkParam)); linkParam.clear();
	//默认RLR
	sType.GCS = EnumArmType_R;
	sType.GCE = EnumArmType_L;
	sType.GCW = EnumArmType_R;
}

int Kinematics::FK(double* dACS, double* dPCS,double* Tr)
{
	double dAcs[6] = { 0 };
	for (int i = 0; i < 6; i++)
	{
		dAcs[i] = *(dACS + i) / 180. * PI;
		*(dPCS + i) = 0;
	}
	double T[4][4] = {
		{1,0,0,0},
		{0,1,0,0},
		{0,0,1,0},
		{0,0,0,1}
	};
	for (int i = 0; i < 6; i++)
	{
		double temp[4][4] = {
		{1,0,0,0},
		{0.1,0,0},
		{0,0,1,0},
		{0,0,0,1}};
		m_Link[i].A(&temp[0][0], dAcs[i]);
		double out[4][4] = {
			{1,0,0,0},
			{0,1,0,0},
			{0,0,1,0},
			{0,0,0,1} };
		MatrixMultiply(&T[0][0], &temp[0][0], &out[0][0], 4);
		for (int j = 0; j < 16; j++)
		{
			*(&T[0][0] + j) = *(&out[0][0] + j);
		}
		T[0][0] = T[0][0];
	}
	//省略使用T叠加工具坐标系
	//t = t * SE3(robot.tool);
	//齐次变换矩阵 (4x4) 进行归一化，确保旋转部分的正交性。省略，
	//trnorm(&T[0][0]);
	*dPCS = T[0][3];
	*(dPCS+1) = T[1][3];
	*(dPCS+2) = T[2][3];
	double r11 = T[0][0];
	double r12 = T[0][1];
	double r13 = T[0][2];
	double r21 = T[1][0];
	double r22 = T[1][1];
	double r23 = T[1][2];
	double r31 = T[2][0];
	double r32 = T[2][1];
	double r33 = T[2][2];

	// 计算欧拉角 (ZYX 旋转顺序)
	*(dPCS + 5) = atan2(r21, r11)/PI*180;                           // 绕 Z 轴的旋转角 A
	*(dPCS + 4) = atan2(-r31, sqrt(r32 * r32 + r33 * r33)) / PI * 180;   // 绕 Y 轴的旋转角 B
	*(dPCS + 3) = atan2(r32, r33) / PI * 180;                            // 绕 X 轴的旋转角 C

	//返回齐次变换矩阵
	for (int i = 0; i < 16; i++)
	{
		*(Tr + i) = *(&T[0][0] + i);
	}

	return 0;
}

int Kinematics_S::IK(double* dPCS, double* dRefACS, double* dACS)
{
	double dTheta[6][8] = { 0 };
	for (uint8_t i = 0; i < 6; i++)
	{
		m_dPCS[i] = *(dPCS + i);
		m_dRefACS[i] = *(dRefACS + i);
	}
	//角度转为弧度
	for (uint8_t i = 0; i < 3; i++)
	{
		//m_dPCS_Rad[i] = Deg2Rad(m_dPCS[3 + i]);
		m_dPCS_Rad[i] = (m_dPCS[3 + i]);
	}

	double matrix_RX[3][3];
	double matrix_RY[3][3];
	double matrix_RZ[3][3];
	RotX(m_dPCS_Rad[0], &matrix_RX[0][0]);
	RotY(m_dPCS_Rad[1], &matrix_RY[0][0]);
	RotZ(m_dPCS_Rad[2], &matrix_RZ[0][0]);
	//三个矩阵相乘得到最后的旋转矩阵
	double T_matrix[3][3];

	MatrixMultiply(&matrix_RZ[0][0], &matrix_RY[0][0], &T_matrix[0][0]);
	MatrixMultiply(&T_matrix[0][0], &matrix_RX[0][0], &RotationMatrix[0][0]);

	//生成齐次变换矩阵
	double T_HTransformationMatrix[4][4] = {
		{RotationMatrix[0][0],RotationMatrix[0][1],RotationMatrix[0][2],m_dPCS[0]},
		{RotationMatrix[1][0],RotationMatrix[1][1],RotationMatrix[1][2],m_dPCS[1]},
		{RotationMatrix[2][0],RotationMatrix[2][1],RotationMatrix[2][2],m_dPCS[2]},
		{0,0,0,1} };
	double a[6] = { m_DH.a1,m_DH.a2, m_DH.a3, m_DH.a4, m_DH.a5, m_DH.a6 };
	double d[6] = { m_DH.d1,0, 0, m_DH.d2 - m_DH.d3 + m_DH.d4, m_DH.d5, m_DH.d6 };
	double alpha[6] = { PI / 2,0,0,PI / 2,-PI / 2,0 };
	m_dPCS[0] = m_dPCS[0];
	//------------------------------Theta 1 ------------------------------
	//Theta1 = 2 solutions, isn't depended on other angles.
	double dP05[4] = {0};
	double m[4] = { 0, 0, -m_DH.d6, 1 };
	multiplyMatrix4x4ByVector4(&T_HTransformationMatrix[0][0], m, dP05);
	dTheta[0][0]= atan2(dP05[1], dP05[0]) + PI / 2;
	dTheta[0][1] = dTheta[0][0];
	if ((abs(dP05[0]-0)>0.0001)|| (abs(dP05[1] - 0) > 0.0001))
	{
		// 计算公共部分
		double numerator = d[3];
		double denom = std::sqrt(dP05[0] * dP05[0] + dP05[1] * dP05[1]);
		double x = numerator / denom;
		// 模拟两种分支计算
		dTheta[0][0] += real_acos(x);
		dTheta[0][1] -= real_acos(x);
	}
	//------------------------------Theta 5 ------------------------------
	//Theta5 = 2 solutions, is depended on theta1 which also have 2 solutions.
	//of solutions = 2 * 2 = 4
	int idx = 0;
	for (size_t i = 0; i < 2; i++)
	{
		double ThetaTemp = (m_dPCS[0] * sin(dTheta[0][i]) - m_dPCS[1] * cos(dTheta[0][i]) - d[3]) / d[5];
		if (ThetaTemp > 1)
		{
			//If this happens all the time, maybe just set theta5 to zero.
			return -1;
		}
		dTheta[4][idx] = 1 * acos(ThetaTemp);
		idx = idx + 1;
		dTheta[4][idx] = -1 * acos(ThetaTemp);
		idx = idx + 1;
	}
	//------------------------------Theta 6 ------------------------------
	//Theta6 = 1 solution, is depended on theta1and theta5 which each have 2 solutions.
	//of solutions = 1 * 2 * 2 = 4
	double I_HTransformationMatrix[4][4] = { 0 };
	inverseMatrix(&T_HTransformationMatrix[0][0], 4, &I_HTransformationMatrix[0][0]);
	double Theta5_x[3] = { I_HTransformationMatrix[0][0],I_HTransformationMatrix[1][0] ,I_HTransformationMatrix[2][0] };
	double Theta5_y[3] = { I_HTransformationMatrix[0][1],I_HTransformationMatrix[1][1] ,I_HTransformationMatrix[2][1] };
	int Theta_R[2] = { 0,2 };
	idx = 0;
	for (size_t i = 0; i < 2; i++)
	{
		dTheta[5][idx] = calculateTheta6(Theta5_x, Theta5_y, dTheta[0][i], dTheta[4][Theta_R[i]]);
		dTheta[5][idx+1] = calculateTheta6(Theta5_x, Theta5_y, dTheta[0][i], dTheta[4][Theta_R[i] + 1]);
		idx = idx + 2;
	}
	//------------------------------Theta 3 ------------------------------
	// We want to find frame T14(to get P14).We can do this by going from frame 1 to base,
	// then go to end effector, frame 5 and at last to frame 4. We can find frame
	// T01, T06, T56and T45.We can then inverse most of theseand multiply them
	// together.
	// Theta3 = 2 solutions, but P14 is depended on theta1, theta5and theta6.
	// of solutions = 2 * 2 * 2 * 1 = 8 solutions

	double t6 = 1;
	double T14_[4][4][8] = {0};
	double P14_[8][3] = {0};
	idx = 0;
	for (size_t i = 0; i < 2; i++)
	{
		double T14[4][4] = { 0 };
		double P14[3] = { 0 };
		int sign = 1;
		double theta3 = 0;
		calculateTheta3(&T_HTransformationMatrix[0][0], alpha, a, d, dTheta[0][i], dTheta[4][Theta_R[i]], dTheta[5][Theta_R[i]], &theta3, &T14[0][0], &P14[0]);
		dTheta[2][idx] = sign * theta3;
		P14_[idx][0] = P14[0];
		P14_[idx][1] = P14[1];
		P14_[idx][2] = P14[2];
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				T14_[i][j][idx] = T14[i][j];
			}
		}
		idx++;
		calculateTheta3(&T_HTransformationMatrix[0][0], alpha, a, d, dTheta[0][i], dTheta[4][Theta_R[i]+1], dTheta[5][Theta_R[i]+1], &theta3, &T14[0][0], &P14[0]);
		dTheta[2][idx] = sign * theta3;
		P14_[idx][0] = P14[0];
		P14_[idx][1] = P14[1];
		P14_[idx][2] = P14[2];
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				T14_[i][j][idx] = T14[i][j];
			}
		}
		idx++;

		sign = -1;
		calculateTheta3(&T_HTransformationMatrix[0][0], alpha, a, d, dTheta[0][i], dTheta[4][Theta_R[i]], dTheta[5][Theta_R[i]], &theta3, &T14[0][0], &P14[0]);
		dTheta[2][idx] = sign * theta3;
		P14_[idx][0] = P14[0];
		P14_[idx][1] = P14[1];
		P14_[idx][2] = P14[2];
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				T14_[i][j][idx] = T14[i][j];
			}
		}
		idx++;
		calculateTheta3(&T_HTransformationMatrix[0][0], alpha, a, d, dTheta[0][i], dTheta[4][Theta_R[i] + 1], dTheta[5][Theta_R[i]+1], &theta3, &T14[0][0], &P14[0]);
		dTheta[2][idx] = sign * theta3;
		P14_[idx][0] = P14[0];
		P14_[idx][1] = P14[1];
		P14_[idx][2] = P14[2];
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				T14_[i][j][idx] = T14[i][j];
			}
		}
		idx++;
	}
	int rearangeIdx[8] ={0, 2, 1, 3, 4, 6, 5, 7};
	double theta3_Copy[8] = {0};
	for (size_t i = 0; i < 8; i++)theta3_Copy[i] = dTheta[2][i];
	double T14_C[4][4][8] = { 0 };
	for (size_t i = 0; i < 4*4*8; i++)*((&T14_C[0][0][0]) + i) = *((&T14_[0][0][0]) + i);
	double P14_C[8][3] = { 0 };
	for (size_t i = 0; i < 3 * 8; i++)*((&P14_C[0][0]) + i) = *((&P14_[0][0]) + i);

	for (size_t i = 0; i < 8; i++)
	{
		theta3_Copy[i] = dTheta[2][rearangeIdx[i]];
		for (size_t s = 0; s < 3; s++)
		{
			P14_C[i][s] = P14_[rearangeIdx[i]][s];
		}
		for (size_t s = 0; s < 4; s++)
		{
			for (size_t m = 0; m < 4; m++)
			{
				T14_C[s][m][i] = T14_[s][m][rearangeIdx[i]];
			}
		}
	}
	for (size_t i = 0; i < 8; i++)dTheta[2][i] = theta3_Copy[i];
	for (size_t i = 0; i < 4 * 4 * 8; i++) *((&T14_[0][0][0]) + i) = *((&T14_C[0][0][0]) + i);
	for (size_t i = 0; i < 3 * 8; i++) *((&P14_[0][0]) + i) = *((&P14_C[0][0]) + i);

	// ------------------------------Theta 2 ------------------------------
	// Theta2 = 1 solution, depends on Theta1, Theta3, Theta5and theta6
	//# of solutions 1 * 2 * 2 * 2 * 1 = 8

	idx = 0;
	double theta2 = { 0 };
	for (size_t i = 0; i < 8; i++)
	{
		double v[2] = { P14_[i][0], P14_[i][2] };
		double P14_xz_length = norm2(v);
		theta2 = atan2(-P14_[i][2], -P14_[i][0]) - asin(-a[2] * sin(dTheta[2][i]) / P14_xz_length);
		dTheta[1][idx] = theta2;
		idx = idx + 1;
	}
	// ------------------------------Theta 4 ------------------------------
	//Since we can't get the transformation matrix T43, because it requires
	//theta4, we can go from frame T23 to T12and then up to T34 using T14
	idx = 0;
	for (size_t i = 0; i < 8; i++)
	{
		double T12[4][4] = {0};
		DH2tform(alpha[0], a[0], d[1], dTheta[1][i], &T12[0][0]);
		double T21[4][4] = { 0 };
		inverseMatrix(T12, T21);

		double T23[4][4] = { 0 };
		DH2tform(alpha[1], a[1], d[2], dTheta[2][i], &T23[0][0]);
		double T32[4][4] = { 0 };
		inverseMatrix(T23, T32);

		double T34[4][4] = { 0 };
		double t[4][4] = { 0 };
		MatrixMultiply(&T32[0][0], &T21[0][0], &t[0][0], 4);
		double tt[4][4] = { 0 };
		for (size_t s = 0; s < 4; s++)
		{
			for (size_t m = 0; m < 4; m++)
			{
				 tt[s][m] = T14_[s][m][i];
			}
		}
		MatrixMultiply(&t[0][0], &tt[0][0], &T34[0][0], 4);
		double X34[3] = { T34[0][0], T34[0][1],  T34[0][2] };
		double theta4 = atan2(X34[1], X34[0]);
		dTheta[3][idx] = -theta4;
		idx = idx + 1;
	}
	double Solution[8][6] = {
		{ dTheta[0][0] ,dTheta[1][0] ,dTheta[2][0] ,dTheta[3][0] ,dTheta[4][0] ,dTheta[5][0]},
		{ dTheta[0][1] ,dTheta[1][1] ,dTheta[2][1] ,dTheta[3][1] ,dTheta[4][1] ,dTheta[5][1]},
		{ dTheta[0][2] ,dTheta[1][2] ,dTheta[2][2] ,dTheta[3][2] ,dTheta[4][2] ,dTheta[5][2]},
		{ dTheta[0][3] ,dTheta[1][3] ,dTheta[2][3] ,dTheta[3][3] ,dTheta[4][3] ,dTheta[5][3]},
		{ dTheta[0][4], dTheta[1][4], dTheta[2][4], dTheta[3][4], dTheta[4][4], dTheta[5][4]},
		{ dTheta[0][5], dTheta[1][5], dTheta[2][5], dTheta[3][5], dTheta[4][5], dTheta[5][5]},
		{ dTheta[0][6], dTheta[1][6], dTheta[2][6], dTheta[3][6], dTheta[4][6], dTheta[5][6]},
		{ dTheta[0][7], dTheta[1][7], dTheta[2][7], dTheta[3][7], dTheta[4][7], dTheta[5][7]},

	};
	///*
	ClosetSolution(&Solution[0][0], 8, m_dWeight, dACS);
	for (size_t s = 0; s < 6; s++)
		*(dACS + s) = Rad2Deg(dTheta[s][0]);
	return 0;
	//*/

	//后面代码有问题，不改了，用E系列的就行了，没什么问题
	//------------------------------Solutions ------------------------------
	double solutions[8][6];
	double faux0 = 0;
	double faux1 = 0;
	double faux2 = 0;
	double sum1[8] = { 0 };
	double sum2[8] = { 0 };
	double sum3[8] = { 0 };
	double sum6[8] = { 0 };
	generatePossibleSolutions(dTheta, dRefACS, &solutions[0][0]);
	for (size_t i = 0;i < 8; i++)
	{
		double q_i[6] = {0};
		for (size_t j = 0; j < 6; j++)q_i[j] = solutions[i][j];
		faux0 = q_i[0] - m_dRefACS[0];
		sum1[i] = faux0 * faux0 * m_dWeight[0];

		faux1 = q_i[1] - m_dRefACS[1];
		sum2[i] = sum1[i] + faux1 * faux1 * m_dWeight[1];

		faux2 = q_i[2] - m_dRefACS[2];
		sum3[i] = sum2[i] + faux2 * faux2 * m_dWeight[2];

		faux0 = q_i[3] - m_dRefACS[3];
		faux1 = q_i[4] - m_dRefACS[4];
		faux2 = q_i[5] - m_dRefACS[5];
		sum6[i] = sum3[i] + m_dWeight[3] * faux0 * faux0 + m_dWeight[4] * faux1 * faux1 + m_dWeight[5] * faux2 * faux2;
	}
	double sumMin = sum6[0];
	size_t out_index = 0;
	for (size_t i = 1; i < 8; i++)
	{
		if (abs(sumMin - sum6[i]) < 1e-6)
		{
			if (abs(sum3[out_index] - sum3[i]) < 1e-6)
			{
				if (abs(sum2[out_index] - sum2[i]) < 1e-6)
				{
					if (sum1[out_index] > sum1[i])
					{
						sumMin = sum6[i];
						out_index = i;
					}
				}
				else if (sum2[out_index] > sum2[i])
				{
					sumMin = sum6[i];
					out_index = i;
				}
			}
			else if (sum3[out_index] > sum3[i])
			{
				sumMin = sum6[i];
				out_index = i;
			}
		}
		else if (sumMin > sum6[i])
		{
			sumMin = sum6[i];
			out_index = i;
		}
	}

	for (size_t s = 0; s < 6; s++)
		*(dACS + s) = Rad2Deg(solutions[out_index][s]);
	return 0;
}
int Kinematics_S::generatePossibleSolutions(double theta[6][8], double* refAcs, double* solutions)
{
	double t[8][6] = {
		{theta[0][0],theta[1][0],theta[2][0], theta[3][0], theta[4][0] ,theta[5][0]},
		{theta[0][0], theta[1][2], theta[2][2], theta[3][2] ,theta[4][1], theta[5][1]},
		{theta[0][1], theta[1][4], theta[2][4], theta[3][4] ,theta[4][2], theta[5][2]},
		{theta[0][1],theta[1][6], theta[2][6], theta[3][6] ,theta[4][3], theta[5][3]},
		{theta[0][0] ,theta[1][1], theta[2][1], theta[3][1], theta[4][0], theta[5][0]},
		{theta[0][0],theta[1][3] ,theta[2][3] ,theta[3][3] ,theta[4][1] ,theta[5][1]},
		{theta[0][1] , theta[1][5] , theta[2][5] , theta[3][5]  ,theta[4][2] , theta[5][2]},
		{theta[0][1] , theta[1][7] , theta[2][7] , theta[3][7] , theta[4][3] , theta[5][3]},
	};
	for (size_t i = 0; i < 8; i++)
	{
		double q[6] = { 0 };
		for (size_t j = 0; j < 6; j++)q[j] = t[i][j];
		for (size_t s = 0; s < 6; s++)
		{
			//如果和参考值q_ref相差大于 pi
			if (abs(q[i] - *(refAcs + i)) > PI)
			{
				if ((*(refAcs + i) - q[i]) > 0)
				{
					q[i] += PI * 2;
				}
				else
				{
					q[i] -= PI * 2;
				}
			}
		}
		for (size_t j = 0; j < 6; j++)t[i][j] = q[j];
	}
	for (size_t i = 0; i < 6 * 8; i++)
	{
		*(solutions + i) = *((&t[0][0]) + i);
	}
	return 0;
}
bool Kinematics_S::calculateTheta3(double* T06, double* alpha, double* a, double* d, double theta1, double theta5, double theta6, double* theta3, double* T14, double* P14)
{
	double T01[4][4] = { 0 };
	double T10[4][4] = { 0 };
	DH2tform(0, 0, *d, theta1, &T01[0][0]);
	inverseMatrix(T01, T10);

	double T45[4][4] = { 0 };
	double T54[4][4] = { 0 };
	DH2tform(*(alpha+3), *(a+3), *(d+4), theta5, &T45[0][0]);
	inverseMatrix(T45, T54);

	double T56[4][4] = { 0 };
	double T65[4][4] = { 0 };
	DH2tform(*(alpha + 4), *(a + 4), *(d + 5), theta6, &T56[0][0]);
	inverseMatrix(T56, T65);

	double x[4][4] = { 0 };
	MatrixMultiply(&T10[0][0], T06, &x[0][0],4);
	double y[4][4] = { 0 };
	MatrixMultiply(&x[0][0], &T65[0][0], &y[0][0], 4);
	MatrixMultiply(&y[0][0], &T54[0][0], T14, 4);
	*P14 = *(T14 + 3);
	*(P14+1) = *(T14 +1*4+ 3);
	*(P14 + 2) = *(T14 + 2 * 4 + 3);
	double v[2] = { *P14, *(P14+2) };
	double P14_xz_length = norm2(v);
	double conditions[2] = { abs(*(a + 1) - *(a + 2)) ,abs(*(a + 1) + *(a + 2)) };
	if ((P14_xz_length > conditions[0]) && (P14_xz_length < conditions[1]))
	{
		*theta3 = acos((P14_xz_length * P14_xz_length - *(a + 1) * *(a + 1) - *(a + 2) * *(a + 2)) / (2 * *(a + 1) * *(a + 2)));
	}
	else
	{
		//*theta3 = acos((P14_xz_length * P14_xz_length - *(a + 1) * *(a + 1) - *(a + 2) * *(a + 2)) / (2 * *(a + 1) * *(a + 2)));
		//printf("calculateTheta3----------error\n");
		//return false;
		double a1 = *(a + 1);
		double a2 = *(a + 2);
		double cos_theta3 = (P14_xz_length * P14_xz_length - a1 * a1 - a2 * a2) / (2 * a1 * a2);
		// 限制 cos_theta3 在合法范围内
		if (cos_theta3 > 1.0)  cos_theta3 = 1.0;
		if (cos_theta3 < -1.0) cos_theta3 = -1.0;
		*theta3 = acos(cos_theta3);
	}
	return true;
}
bool Kinematics_S::DH2tform(double alpha, double a, double d, double theta, double* out)
{
	eye(out,4);
	//Row 1
	*(out+0) = cos(theta);
	*(out + 1) = -sin(theta);
	*(out + 2) = 0;
	*(out + 3) = a;

	//Row 2
	*(out+4*1 + 0) = sin(theta) * cos(alpha);
	*(out + 4 * 1 + 1) = cos(theta) * cos(alpha);
	*(out + 4 * 1 + 2 ) = -sin(alpha);
	*(out + 4 * 1 + 3) = -sin(alpha) * d;

	//Row 3
	*(out + 4 * 2 + 0) = sin(theta) * sin(alpha);
	*(out + 4 * 2 + 1) = cos(theta) * sin(alpha);
	*(out + 4 * 2 + 2) = cos(alpha);
	*(out + 4 * 2 + 3) = cos(alpha) * d;
	return true;
}
double Kinematics_S::calculateTheta6(double* x, double* y, double theta1, double theta5)
{
	if (sin(theta5) != 0)
	{
		double leftNumerator = -(*(x+1)) * sin(theta1) + (*(y + 1)) * cos(theta1);
		double rightNumerator = (*x) * sin(theta1) - (*y) * cos(theta1);
		double denominator = sin(theta5);
		return atan2(leftNumerator / denominator, rightNumerator / denominator);
	}
	return 0;
}
int Kinematics_E::IK(double* dPCS, double* dRefACS, double* dACS)
{
	for (uint8_t i = 0; i < 6; i++)
	{
		m_dPCS[i] = *(dPCS + i);
		m_dRefACS[i] = *(dRefACS + i);
	}
	//角度转为弧度
	for (uint8_t i = 0; i < 3; i++)
	{
		//m_dPCS_Rad[i] = Deg2Rad(m_dPCS[3 + i]);
		m_dPCS_Rad[i] = m_dPCS[3 + i];
	}

	double matrix_RX[3][3];
	double matrix_RY[3][3];
	double matrix_RZ[3][3];
	RotX(m_dPCS_Rad[0], &matrix_RX[0][0]);
	RotY(m_dPCS_Rad[1], &matrix_RY[0][0]);
	RotZ(m_dPCS_Rad[2], &matrix_RZ[0][0]);
	//三个矩阵相乘得到最后的旋转矩阵
	double T_matrix[3][3];

	MatrixMultiply(&matrix_RZ[0][0], &matrix_RY[0][0], &T_matrix[0][0]);
	MatrixMultiply(&T_matrix[0][0], &matrix_RX[0][0], &RotationMatrix[0][0]);
	//MatrixMultiply(&matrix_RX[0][0], &matrix_RY[0][0], &T_matrix[0][0]);
	//MatrixMultiply(&T_matrix[0][0], &matrix_RZ[0][0], &RotationMatrix[0][0]);

	//生成齐次变换矩阵
	double T_HTransformationMatrix[4][4] = {
		{RotationMatrix[0][0],RotationMatrix[0][1],RotationMatrix[0][2],m_dPCS[0]},
		{RotationMatrix[1][0],RotationMatrix[1][1],RotationMatrix[1][2],m_dPCS[1]},
		{RotationMatrix[2][0],RotationMatrix[2][1],RotationMatrix[2][2],m_dPCS[2]},
		{0,0,0,1} };
	//生成齐次变换矩阵保存
	for(int i= 0;i<16;i++)
	{
		*(&HTransformationMatrix[0][0] + i) = *(&T_HTransformationMatrix[0][0] + i);
	}
	//ELFIN
	double q1[6]; int arm_type1[3] = { 1,1,1 };
	double q2[6]; int arm_type2[3] = { 1,-1,1 };
	double q3[6]; int arm_type3[3] = { 1,1,-1 };
	double q4[6]; int arm_type4[3] = { 1,-1,-1 };
	double q5[6]; int arm_type5[3] = { -1,1,1 };
	double q6[6]; int arm_type6[3] = { -1,-1,1 };
	double q7[6]; int arm_type7[3] = { -1,1,-1 };
	double q8[6]; int arm_type8[3] = { -1,-1,-1 };
	ElfinIK(arm_type1, q1);
	ElfinIK(arm_type2, q2);
	ElfinIK(arm_type3, q3);
	ElfinIK(arm_type4, q4);
	ElfinIK(arm_type5, q5);
	ElfinIK(arm_type6, q6);
	ElfinIK(arm_type7, q7);
	ElfinIK(arm_type8, q8);
	double Solution[8][6] = {
		{ q1[0] ,q1[1] ,q1[2] ,q1[3] ,q1[4] ,q1[5]},
		{ q2[0] ,q2[1] ,q2[2] ,q2[3] ,q2[4] ,q2[5]},
		{ q3[0] ,q3[1] ,q3[2] ,q3[3] ,q3[4] ,q3[5]},
		{ q4[0] ,q4[1] ,q4[2] ,q4[3] ,q4[4] ,q4[5]},
		{ q5[0] ,q5[1] ,q5[2] ,q5[3] ,q5[4] ,q5[5]},
		{ q6[0] ,q6[1] ,q6[2] ,q6[3] ,q6[4] ,q6[5]},
		{ q7[0] ,q7[1] ,q7[2] ,q7[3] ,q7[4] ,q7[5]},
		{ q8[0] ,q8[1] ,q8[2] ,q8[3] ,q8[4] ,q8[5]}
	};
	ClosetSolution(&Solution[0][0], NUMOFSOLVED_ELFIN,m_dWeight, dACS);
	for (int i = 0; i < 6; i++)
		*(dACS + i) = Rad2Deg(*(dACS + i));
	return 0;
}


int Kinematics_E::ElfinIK(int* arm_type, double* dACS)
{
	int GCS = *arm_type;
	int GCE = *(arm_type +1);
	int GCW = *(arm_type +2);
	double q[6] = { 0 };
	double wx = HTransformationMatrix[0][3] - HTransformationMatrix[0][2] * m_DH.d6;
	double wy = HTransformationMatrix[1][3] - HTransformationMatrix[1][2] * m_DH.d6;
	double wz = HTransformationMatrix[2][3] - HTransformationMatrix[2][2] * m_DH.d6;
	//q1
	if ((wx * wx + wy * wy) < 1e-12)
		q[0] = m_dRefACS[0];
	else
		q[0] = atan2(wy, wx) + (1.000 + GCS) / 2.000 * PI;

	//q2
	double r = sqrt(wx * wx + wy * wy + (wz - m_DH.d1) * (wz - m_DH.d1));

	if (abs(r * r - (m_DH.a2 + m_DH.d4) * (m_DH.a2 + m_DH.d4)) < 1e-12)
		r = m_DH.a2 + m_DH.d4;
	q[1] = GCS * atan2(sqrt(wx * wx + wy * wy), wz - m_DH.d1) + GCE * acos((m_DH.a2 * m_DH.a2 + r * r - m_DH.d4 * m_DH.d4) / (2.000 * m_DH.a2 * r));


	//q3
	double cTri = (m_DH.a2 * m_DH.a2 + m_DH.d4 * m_DH.d4 - r * r) / (2 * m_DH.a2 * m_DH.d4);//三角形内角余弦

	if (abs(cTri - 1) < 1e-12)
		cTri = 1;
	else if (abs(cTri + 1) < 1e-12)
		cTri = -1;
	// 超出工作空间，返回无解状态
	if (cTri > 1 || cTri < -1)
	{
		for (int i = 0; i < 6; i++)
			q[i] = 0;
		return -1;
	}
	q[2] = GCE * (PI - acos(cTri));

	//q5
	double c5 = HTransformationMatrix[2][2] * cos(q[1] - q[2]) -
		(HTransformationMatrix[0][2] * cos(q[0]) +
			HTransformationMatrix[1][2] * sin(q[0])) * sin(q[1] - q[2]);
	if (abs(c5 - 1) < 1e-12)
		c5 = 1;
	else if (abs(c5 + 1) < 1e-12)
		c5 = -1;
	q[4] = GCW * acos(c5);

	//q4 / q6
	if (abs(sin(q[4])) < 1e-6)
	{ // 腕部奇异时，q4保持参考角度不变
		double s46 = 0;
		double c46 = 0;
		s46 = HTransformationMatrix[1][0] * cos(q[0]) - HTransformationMatrix[0][0] * sin(q[0]);
		c46 = HTransformationMatrix[1][1] * cos(q[0]) - HTransformationMatrix[0][1] * sin(q[0]);
		q[3] = m_dRefACS[3];
		q[5] = atan2(s46, c46) - q[3];
	}
	else
	{
		double k1 = HTransformationMatrix[1][2] * cos(q[0]) - HTransformationMatrix[0][2] * sin(q[0]);
		double k2 = HTransformationMatrix[2][2] * sin(q[1] - q[2]) + (HTransformationMatrix[0][2] * cos(q[0]) + HTransformationMatrix[1][2] * sin(q[0])) * cos(q[1] - q[2]);
		q[3] = atan2(k1 * sin(q[4]), k2 * sin(q[4]));
		//q6
		double k3 = HTransformationMatrix[2][1] * cos(q[1] - q[2]) - (HTransformationMatrix[0][1] * cos(q[0]) + HTransformationMatrix[1][1] * sin(q[0])) * sin(q[1] - q[2]);
		double k4 = HTransformationMatrix[2][0] * cos(q[1] - q[2]) - (HTransformationMatrix[0][0] * cos(q[0]) + HTransformationMatrix[1][0] * sin(q[0])) * sin(q[1] - q[2]);
		q[5] = atan2(k3 * sin(q[4]), -k4 * sin(q[4]));
	}
		//如果大于PI 设置到 pi以内
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 确定关节 1 - 6 的范围 % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	for (int i = 0; i < 6; i++)
	{
		while (abs(q[i] - m_dRefACS[i]) > PI)
		{
			if ((m_dRefACS[i] - q[i]) > 0)
				q[i] = q[i] + (2 * PI);
			else
				q[i] = q[i] - (2 * PI);
		}
		*(dACS + i) = q[i];
	}
	return true;

}


int Kinematics::ClosetSolution(double* solutions,int NUMOFSOLVED, double* weight, double* dACS)
{
	double faux0 = 0;
	double faux1 = 0;
	double faux2 = 0;
	double sum1[8] = { 0 };
	double sum2[8] = { 0 };
	double sum3[8] = { 0 };
	double sum6[8] = { 0 };
	for (int i = 0; i < NUMOFSOLVED; i++)
	{
		double q_i[6] = { 0 };
		//拿出第一行
		for (int j = 0; j < 6; j++)
			q_i[j] = *(solutions+(i*6) +j);
		faux0 = q_i[0] - m_dRefACS[0];
		sum1[i] = faux0 * faux0 * (*weight);

		faux1 = q_i[1] - m_dRefACS[1];
		sum2[i] = sum1[i] + faux1 * faux1 * (*(weight+1));

		faux2 = q_i[2] - m_dRefACS[2];
		sum3[i] = sum2[i] + faux2 * faux2 * (*(weight + 2));

		faux0 = q_i[3] - m_dRefACS[3];
		faux1 = q_i[4] - m_dRefACS[4];
		faux2 = q_i[5] - m_dRefACS[5];

		sum6[i] = sum3[i] + (*(weight + 3)) * faux0 * faux0 + (*(weight + 4)) * faux1 * faux1 + (*(weight + 5)) * faux2 * faux2;
	}
	// 默认去第一个解的值作为最小代价
	double SumMin = sum6[0];
	int out_index = 1;
	for (int i = 2; i < NUMOFSOLVED; i++)
	{
		if (abs(SumMin - sum6[i - 1]) < 1e-6) 
		{
			if (abs(sum3[out_index-1] - sum3[i - 1]) < 1e-6)
			{
				if (abs(sum2[out_index-1] - sum2[i - 1]) < 1e-6)
				{
					if (sum1[out_index-1] > sum1[i - 1])
					{
						SumMin = sum6[i-1];
						out_index = i;
					}
				}
				else if (sum2[out_index-1] > sum2[i-1])
				{
					SumMin = sum6[i - 1];
					out_index = i;
				}
			}
			else if (sum3[out_index - 1] > sum3[i - 1])
			{
				SumMin = sum6[i - 1];
				out_index = i;
			}
		}
		else if (SumMin > sum6[i-1])
		{
			SumMin = sum6[i - 1];
			out_index = i;
		}
	}
	for (int j = 0; j < 6; j++)
	{
		*(dACS + j) = *(solutions + 6 * (out_index - 1) + j);
	}
	return 0;
}