#ifndef COMMON_H
#define COMMON_H


#include <math.h>

#ifndef M_PI
#define M_PI std::acos(-1)
#endif // !M_PI

/// <summary>
/// ��ת��˳��
/// </summary>
enum class E_ROTATION_SEQUENCE
{
	E_SEQ_XYZ = 0,
	E_SEQ_ZYX = 1
};

/// <summary>
/// ��ת�ǽǶȵ�λ���ͣ�degree/radian��
/// </summary>
enum class E_ANGLE_TYPE
{
	E_TYPE_DEGREE = 0,
	E_TYPE_RADIAN = 1
};

/// <summary>
/// λ�˽ṹ��
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
