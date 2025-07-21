#include"Kinematics.h"
#include<stdio.h>
#include <iostream>
#include <iomanip>
#include <windows.h>  // Sleep()
#include <cstdlib>    // system("cls")
int main()
{
	/*
	double PCS[6] = { 420 ,0,415,180,0,180 };
	double TK[4][4] = { 0 };
	
	ST_DH dh_e;
	dh_e.d1 = 0.220;
	dh_e.d4 = 0.420;
	dh_e.d6 = 0.185;
	dh_e.a2 = 0.380;
	Kinematics* kin_e = Kinematics::CreateKinematics(0, dh_e);
	for (int i = 0; i < 6; i++)
		PCS[i] = PCS[i] / 1000.000;


	double refAcs[6] = { 10, 20, 90, 20, 77, 60 };
	double ACS[6] = { 0 };
	kin_e->FK(refAcs, PCS, &TK[0][0]);

	double PCS1[6] = { 0.42 ,0,0.415,180,0,180 };
	double pp[6] = { 0,0,0,0,0,0 };
	kin_e->IK(PCS1, pp, ACS);
	std::cout << "MoveLTest" << std::endl;
	std::cout << "CmdAcs = [ ";
	for (int i = 0; i < 6; ++i)
	{
		std::cout << std::fixed << std::setprecision(4) << ACS[i];
		if (i != 5) std::cout << ", ";
	}
	std::cout << " ]" << std::endl;

	std::cout << "CmdPCS = [ ";
	for (int i = 0; i < 6; ++i)
	{
		if (i < 3)
		{
			std::cout << std::fixed << std::setprecision(4) << PCS1[i] * 1000;
			if (i != 5) std::cout << ", ";
		}
		else
		{
			std::cout << std::fixed << std::setprecision(4) << PCS1[i];
			if (i != 5) std::cout << ", ";
		}
	}
	std::cout << " ]" << std::endl;
	*/





	

	///*SÏµÁÐ
	ST_DH dh_s;
	dh_s.d1 = 0.171;
	dh_s.d2 = 0.2295;
	dh_s.d3 = 0.1885;
	dh_s.d4 = 0.138;
	dh_s.d5 = 0.138;
	dh_s.d6 = 0.1257;
	dh_s.a2 = -0.85;
	dh_s.a3 = -0.712;
	double PCS1[6] = { 0 };
	double TK1[4][4] = { 0 };
	Kinematics* kin_s = Kinematics::CreateKinematics(1, dh_s);
	double Acs[6] = { 30, -90, 60, -90, 90, 70 };
	kin_s->FK(Acs, PCS1, &TK1[0][0]);
	double refAcs[6] = { 0, 0, 0, 0, 0, 0 };
	double outAcs[6] = { 0, 0, 0, 0, 0, 0 };
	double PCS2[6] = { -0.493570 ,-0.491654,1.554859,-11.170,28.024,7.204 };
	//double PCS2[6] = { -1.394604 ,-0.500879,0.170497,-177.081,-51.074,-126.302 };
	kin_s->IK(PCS2, refAcs, outAcs);
	while (PCS2[0] < -0.100)
	{
		kin_s->IK(PCS2, refAcs, outAcs);
		std::cout << "MoveLTest"<<std::endl;
		std::cout << "CmdAcs = [ ";
		for (int i = 0; i < 6; ++i) 
		{
			std::cout << std::fixed << std::setprecision(4) << outAcs[i];
			if (i != 5) std::cout << ", ";
		}
		std::cout << " ]" << std::endl;

		std::cout << "CmdPCS = [ ";
		for (int i = 0; i < 6; ++i) 
		{
			if (i < 3)
			{
				std::cout << std::fixed << std::setprecision(4) << PCS2[i] * 1000;
				if (i != 5) std::cout << ", ";
			}
			else
			{
				std::cout << std::fixed << std::setprecision(4) << PCS2[i] ;
				if (i != 5) std::cout << ", ";
			}
		}
		std::cout << " ]" << std::endl;
		PCS2[0] = PCS2[0] + 0.001;
		Sleep(50);
		system("cls");
	}
	//*/
	printf("helloWorld");
}