#pragma once
#include"MathAlgorithms.h"
#include<vector>
#include<string>
class Link
{
public:
    Link(std::vector<double>LDHvar);
    ~Link();
    int A(double* Acs,double M);
private:
    // 运动学参数
    double theta;  // 关节角度，若未指定则默认为转动关节
    double d;      // 关节位移，若未指定则默认为平移关节
    double alpha;  // 关节扭转角，默认为0
    double a;      // 链节偏移量，默认为0
    std::string jointtype = "R"; // 关节类型，默认为“R”表示转动关节
    std::string convention;
    std::string type;
    double dh[20];        // 运动学模型：0为标准D-H参数，1为修改版D-H参数
    double offset = 0.0; // 关节变量偏移，默认为0
    std::vector<double> qlim; // 关节限位（默认为空，即无限位）

    // 动力学参数
    double m = 0.0;              // 链节质量
    std::vector<double> r = { 0.0, 0.0, 0.0 }; // 链节质心位置（3x1）
    std::vector<std::vector<double>> I = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} }; // 链节惯性矩阵（3x3）
    double Jm = 0.0;             // 马达惯性（参考于马达）
    double G = 1.0;              // 齿轮比（默认为1）
    double B = 0.0;              // 关节摩擦（参考于马达，默认为0）
    std::vector<double> Tc = { 0.0, 0.0 }; // 库仑摩擦（1x1 或 2x1，默认为[0, 0]）
    double flip;
    double mdh;

};


struct ST_SerialLinkData
{
    std::vector<Link> links;
    int n = 0;
    std::string name;
    std::string comment;
    std::string manufacturer;
    double base[4][4] = {
        {1, 0, 0 ,0},
        {0, 1, 0 ,0},
        {0, 0, 1 ,0},
        {0, 0, 0 ,1} };
    double tool[4][4] = {
        {1, 0, 0 ,0},
        {0, 1, 0 ,0},
        {0, 0, 1 ,0},
        {0, 0, 0 ,1} };
    double gravity[3] = { 0,0,9.81 };
    bool opt_modified;
};

class SerialLink
{
public:
    SerialLink(std::vector<Link> m_Link, std::string name, std::string model);
    ~SerialLink();

private:
    ST_SerialLinkData m_R, m_Default, m_Opt;
};

