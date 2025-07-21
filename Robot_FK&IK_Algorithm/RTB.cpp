#include"RTB.h"
#include<vector>

Link::Link(std::vector<double> LDHvar)
{
	for (short i = 0; i < LDHvar.size(); i++)
		dh[i] = LDHvar[i];
	if (LDHvar.size() > 4)
		printf("RTB:Link:badarg', 'must provide params (theta d a alpha)\n");
	this->theta = dh[0];
	this->d = dh[1];
	this->a = dh[2];
	this->alpha = dh[3];
	this->jointtype = "R";
	this->offset = LDHvar[0];
	this->flip = false;
	this->mdh = 0;
	if (LDHvar.size() > 5)
		if (dh[4] == 1)
			this->jointtype = "P";
	if (LDHvar.size() == 6)
		this->offset = dh[5];
	if (LDHvar.size() > 6)
	{
		if (dh[4] > 0)
			this->jointtype = "P";
		else
			this->jointtype = "R";
		//动力学赋值部分不写了，反正不用
	}
	if ("modified" == this->convention)
		this->mdh = 1;
	else
		this->mdh = 0;

}

Link::~Link()
{
}

int Link::A(double* Acs, double q)
{
	double sa = sin(alpha);
	double ca = cos(alpha);
	q = q + offset;
	double st = sin(q);
	double ct = cos(q);
	double m_m[4][4] = { 
		{ct,-st*ca,st*sa,a*ct},
		{st,ct*ca,-ct*sa,a*st},
		{0,sa,ca,d},
		{0,0,0,1}
	};
	for (int i = 0; i < 16; i++)
	{
		*(Acs + i) = *(&m_m[0][0] + i);
	}
	return 0;
}


SerialLink::SerialLink(std::vector<Link> m_Link, std::string name, std::string model)
{
	m_Opt.name = name;
	m_R.links = m_Link;
	m_R.n = m_Link.size();

}

SerialLink::~SerialLink()
{
}