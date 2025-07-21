#pragma once
struct ST_DH
{
    double a1 = 0;
    double a2 = 0;
    double a3 = 0;
    double a4 = 0;
    double a5 = 0;
    double a6 = 0;
    double d1 = 0;
    double d2 = 0;
    double d3 = 0;
    double d4 = 0;
    double d5 = 0;
    double d6 = 0;
};

enum EnumArmType {
    EnumArmType_R = 82,
    EnumArmType_L = 76,
};
struct ST_ArmType
{
    EnumArmType GCS;
    EnumArmType GCE;
    EnumArmType GCW;
};