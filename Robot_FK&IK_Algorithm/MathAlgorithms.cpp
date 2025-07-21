#include"MathAlgorithms.h"
#include<math.h>
double Deg2Rad(double deg)
{
	return (PI / 180) * deg;
}

double Rad2Deg(double rad)
{
	return rad / (PI / 180);
}
//matlab函数
double cosd(double data)
{
	return cosf(data * PI / 180);
}

double sind(double data)
{
	return sinf(data * PI / 180);
}

int  RotX(double rad, double* matrix)
{
	double T_matrix[3][3] = {
		{1,0,0},
		{0,cosd(rad),-sind(rad)},
		{0,sind(rad),cosd(rad)}
	};
	double* ptr = &T_matrix[0][0];
	//矩阵值同步
	for (int i = 0; i < 9; i++)
		*(matrix + i) = *(ptr + i);
	return 0;
}

int  RotY(double rad, double* matrix)
{
	double T_matrix[3][3] = {
		{cosd(rad),0,sind(rad)},
		{0,1,0},
		{-sind(rad),0,cosd(rad)}
	};
	double* ptr = &T_matrix[0][0];
	for (int i = 0; i < 9; i++)
		*(matrix + i) = *(ptr + i);
	return 0;
}
int  RotZ(double rad, double* matrix)
{
	double T_matrix[3][3] = {
	{cosd(rad),-sind(rad),0},
	{sind(rad),cosd(rad),0},
	{0,0,1}
	};
	double* ptr = &T_matrix[0][0];
	for (int i = 0; i < 9; i++)
		*(matrix + i) = *(ptr + i);
	return 0;
}


int MatrixMultiply(const double* A, const double* B, double* C, int size) {
	// 初始化结果矩阵 C 为 0
	std::fill(C, C + size * size, 0.0);
	// 执行矩阵乘法 C = A * B
	for (int i = 0; i < size; ++i) {      // 遍历 C 的行
		for (int j = 0; j < size; ++j) {  // 遍历 C 的列
			for (int k = 0; k < size; ++k) {  // A 的列 & B 的行
				C[i * size + j] += A[i * size + k] * B[k * size + j];
			}
		}
	}
	return 0;
}



void trnorm(double* T) {
	assert(T != nullptr && "输入矩阵指针不能为空");

	// 提取旋转矩阵的列向量 o, a
	double o[3] = { T[0 + 1 * 4], T[1 + 1 * 4], T[2 + 1 * 4] }; // 第二列
	double a[3] = { T[0 + 2 * 4], T[1 + 2 * 4], T[2 + 2 * 4] }; // 第三列
	double n[3]; // 第一列 (通过叉积计算)

	// 1. 计算 n = o x a (手动计算叉积，不调用其他函数)
	n[0] = o[1] * a[2] - o[2] * a[1];
	n[1] = o[2] * a[0] - o[0] * a[2];
	n[2] = o[0] * a[1] - o[1] * a[0];

	// 2. 重新计算 o = a x n (保持正交性)
	o[0] = a[1] * n[2] - a[2] * n[1];
	o[1] = a[2] * n[0] - a[0] * n[2];
	o[2] = a[0] * n[1] - a[1] * n[0];

	// 3. 归一化 n, o, a 向量
	double norm_n = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
	double norm_o = std::sqrt(o[0] * o[0] + o[1] * o[1] + o[2] * o[2]);
	double norm_a = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);

	if (norm_n > 1e-12) {
		n[0] /= norm_n; n[1] /= norm_n; n[2] /= norm_n;
	}
	if (norm_o > 1e-12) {
		o[0] /= norm_o; o[1] /= norm_o; o[2] /= norm_o;
	}
	if (norm_a > 1e-12) {
		a[0] /= norm_a; a[1] /= norm_a; a[2] /= norm_a;
	}

	// 4. 将计算结果写回齐次变换矩阵 T 中 (只修改旋转部分)
	T[0] = n[0]; T[4] = o[0]; T[8] = a[0];
	T[1] = n[1]; T[5] = o[1]; T[9] = a[1];
	T[2] = n[2]; T[6] = o[2]; T[10] = a[2];

	// 5. 保持齐次变换矩阵的最后一行为 [0, 0, 0, 1]
	T[3] = 0.0;
	T[7] = 0.0;
	T[11] = 0.0;
	T[12] = 0.0;
	T[13] = 0.0;
	T[14] = 0.0;
	T[15] = 1.0;
}

void multiplyMatrix4x4ByVector4(const double* T, const double* v, double* result)
{
	for (int i = 0; i < 4; ++i) {
		result[i] = 0.0;
		for (int j = 0; j < 4; ++j) {
			result[i] += T[i * 4 + j] * v[j];  // T按行存储的一维数组
		}
	}
}

// 计算 real(acos(complex(x)))
double real_acos(double x) 
{
	if (x >= -1.0 && x <= 1.0) {
		return acos(x);  // 纯实数
	}
	else {
		// 超过 [-1,1]，返回复数的实部
		return std::log(x + std::sqrt(x * x - 1));
	}
}

// 矩阵求逆：高斯-约旦消元法
bool inverseMatrix(const double* A, int n, double* A_inv)
{
	
	double temp[10][10];

	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < n; ++j) {
			temp[i][j] = A[i * n + j];
			temp[i][j + n] = (i == j) ? 1.0 : 0.0;
		}
	}
	for (int i = 0; i < n; ++i) {
		if (fabs(temp[i][i]) < 1e-12) return false;
		double pivot = temp[i][i];
		for (int j = 0; j < 2 * n; ++j)
			temp[i][j] /= pivot;
		for (int k = 0; k < n; ++k) {
			if (k == i) continue;
			double factor = temp[k][i];
			for (int j = 0; j < 2 * n; ++j) {
				temp[k][j] -= factor * temp[i][j];
			}
		}
	}
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < n; ++j) {
			A_inv[i * n + j] = temp[i][j + n];
		}
	}
	return true;
	
}
void inverseMatrix(const double A[4][4], double A_inv[4][4])
{
	double R[3][3];
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			R[i][j] = A[i][j];
		}
	}
	double R_T[3][3];
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			R_T[i][j] = R[j][i];
		}
	}
	double t[3];
	for (int i = 0; i < 3; ++i)
	{
		t[i] = A[i][3];
	}
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			A_inv[i][j] = R_T[i][j];
		}
	}
	for (int i = 0; i < 3; ++i)
	{
		A_inv[i][3] = 0;
		for (int j = 0; j < 3; ++j)
		{
			A_inv[i][3] -= R_T[i][j] * t[j];
		}
	}
	A_inv[3][0] = 0;
	A_inv[3][1] = 0;
	A_inv[3][2] = 0;
	A_inv[3][3] = 1;
}


double norm3(const double* a, const double* b) 
{
	double sum = 0.0;
	for (int i = 0; i < 3; ++i) {
		double diff = a[i] - b[i];
		sum += diff * diff;
	}
	return std::sqrt(sum);
}
double norm2(const double* v)
{
	return std::sqrt(v[0] * v[0] + v[1] * v[1]);
}

bool eye(double* m, int k)
{
	for (size_t i = 0; i < k; i++)
	{
		for (size_t j = 0; j < k; j++)
		{
			if (j == i)
			{
				*(m + i * k + j) = 1;
			}
			else
			{
				*(m + i * k + j) = 0;
			}
		}
	}
	return true;
}
