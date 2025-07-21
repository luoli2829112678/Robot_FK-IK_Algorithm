#pragma once
#include"RTBDef.h"
#include <iomanip>
#include <cassert>
#include <complex>
#define PI 3.1415926

double Deg2Rad(double deg);
double Rad2Deg(double rad);
//matlabº¯Êý·½·¨
double cosd(double data);
double sind(double data);

int MatrixMultiply(const double* A, const double* B, double* C, int size = 3);
void multiplyMatrix4x4ByVector4(const double* T, const double* v, double* result);
bool inverseMatrix(const double* A, int n, double* A_inv);
void inverseMatrix(const double A[4][4], double A_inv[4][4]);
int  RotX(double rad, double* matrix);
int  RotY(double rad, double* matrix);
int  RotZ(double rad, double* matrix);
void trnorm(double* T);
double real_acos(double x);
double norm3(const double* a, const double* b);
double norm2(const double* v);
bool eye(double* m,int k);