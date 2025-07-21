#pragma once
#include"RTB.h"
class Kinematics
{
public:
	Kinematics(int nModel ,ST_DH dh);
	~Kinematics();
	virtual int IK(double* dPCS, double* dRefACS, double* dACS) = 0;
	int FK(double* dACS, double* dPCS, double* Tr);
	static Kinematics* CreateKinematics(int model, ST_DH dh);

protected:
	Kinematics *m_Obj;
	int m_nRobotType = 0;
	ST_DH m_DH;
	ST_ArmType sType;
	double m_dPCS[6];
	double m_dRefACS[6];
	double m_dPCS_Rad[3];
	std::vector<Link> m_Link;
	double RotationMatrix[3][3];
	double HTransformationMatrix[4][4];
	double m_dWeight[6] = { 6, 5, 4, 0.1, 1, 0.1 };
	int NUMOFSOLVED_ELFIN = 8;
	int ClosetSolution(double* solutions, int NUMOFSOLVED, double* weight, double* dACS);
};

class Kinematics_E:public Kinematics
{
public:
	Kinematics_E( ST_DH dh);
	int IK(double* dPCS, double* dRefACS, double* dACS);
private:
	int ElfinIK(int*, double* b);
};

class Kinematics_S :public Kinematics
{
public:
	Kinematics_S(ST_DH dh);
	int IK(double* dPCS, double* dRefACS, double* dACS);
private:
	double m_dWeight[6] = { 1,1, 1, 1, 1, 1 };
	double calculateTheta6(double* x,double* y,double theta1,double theta5);
	bool calculateTheta3(double*T06,double *alpha,double*a,double*d,double theta1,double theta5,double theta6,double* theta3,double*P14,double*T14);
	//out = 4*4
	bool DH2tform(double alpha, double a, double d, double theta, double* out);
	int generatePossibleSolutions(double theta[6][8], double* refAcs, double* solutions);
};