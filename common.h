#ifndef COMMON_H
#define COMMON_H

#include <math.h>

#ifndef M_PI
#define M_PI std::acos(-1)
#endif // !M_PI

/// <summary>
/// 旋转角顺序
/// </summary>
enum class E_ROTATION_SEQUENCE
{
	E_SEQ_XYZ = 0,
	E_SEQ_ZYX = 1
};

/// <summary>
/// 旋转角角度单位类型（degree/radian）
/// </summary>
enum class E_ANGLE_TYPE
{
	E_TYPE_DEGREE = 0,
	E_TYPE_RADIAN = 1
};

/// <summary>
/// 数值容差常量
/// 把「输入合法性校验容差」与「算法分支阈值」分开命名，避免同一个数字承担两种语义
/// </summary>
namespace TRANSFORM_TOL
{
	/// 输入合法性校验：旋转矩阵正交性 / 行列式的允许偏差（容忍 float 精度与标定误差）
	constexpr double CHECK_ORTHOGONAL = 1e-6;

	/// 数值分支：旋转向量模长小于该值时视为零旋转（只与 double 精度有关）
	constexpr double BRANCH_ZERO_ANGLE = 1e-12;

	/// 数值分支：旋转角与 pi 的差值小于该值时视为恰好 pi
	constexpr double BRANCH_NEAR_PI = 1e-6;

	/// 数值分支：从 R + I 提取旋转轴时，判定某列可用的最小范数
	constexpr double BRANCH_AXIS_ELEMENT = 1e-6;

	/// 数值分支：欧拉角解算中万向锁判定（与 pi/2 的允许差值，弧度）
	constexpr double BRANCH_GIMBAL_LOCK = 1e-6;
}

/// <summary>
/// 位姿结构体
/// </summary>
struct S_POSE
{
public:
	explicit S_POSE(double x, double y, double z, double rx, double ry, double rz)
		:X(x), Y(y), Z(z), Rx(rx), Ry(ry), Rz(rz) {};
public:
	double X;	double Y;	double Z;
	double Rx;	double Ry;	double Rz;
};

#endif // !COMMON_H
