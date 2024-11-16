/*****************************************************************************
*  @file     transformation_eg.cpp                                           *
*  @brief    用Eigen实现三维空间中位姿与矩阵之间的变换                          *
*                                                                            *
*  @author   Chan_XM                                                         *
*  @email    chan_xm14@163.com			                                     *
*  @version  V1.0														     *
*  @date     2024/11/16													     *
*  @license  NON															 *
*****************************************************************************/

#include "transformation_eg.h"
#include <iostream>

double CTransformation::degree2Radian(const double& degree)
{
	return degree * M_PI / 180;
}

double CTransformation::radian2Degree(const double& radian)
{
	return radian * 180 / M_PI;
}

Eigen::Matrix3d CTransformation::rotDegree2Matrix(double rx, double ry, double rz, const E_ROTATION_SEQUENCE& rotSeq)
{
	Eigen::Matrix3d rotationMatrix;

	rx = degree2Radian(rx);
	ry = degree2Radian(ry);
	rz = degree2Radian(rz);

	Eigen::AngleAxisd rotX(Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()));
	Eigen::AngleAxisd rotY(Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()));
	Eigen::AngleAxisd rotZ(Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()));

	if (rotSeq == E_ROTATION_SEQUENCE::E_SEQ_XYZ)
	{
		rotationMatrix = rotZ * rotY * rotX;
	}
	else
	{
		rotationMatrix = rotX * rotY * rotZ;
	}

	return rotationMatrix;
}

Eigen::Matrix3d CTransformation::rotRadian2Matrix(double rx, double ry, double rz, const E_ROTATION_SEQUENCE& rotSeq)
{
	Eigen::Matrix3d rotationMatrix;

	Eigen::AngleAxisd rotX(Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()));
	Eigen::AngleAxisd rotY(Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()));
	Eigen::AngleAxisd rotZ(Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()));

	if (rotSeq == E_ROTATION_SEQUENCE::E_SEQ_XYZ)
	{
		rotationMatrix = rotZ * rotY * rotX;
	}
	else
	{
		rotationMatrix = rotX * rotY * rotZ;
	}

	return rotationMatrix;
}

Eigen::Vector3d CTransformation::rotMatrix2RotDegree(const Eigen::Matrix3d& matrix, const E_ROTATION_SEQUENCE& rotSeq)
{
	double R11, R12, R13, R21, R22, R23, R31, R32, R33;
	R11 = matrix(0, 0);		R12 = matrix(0, 1);		R13 = matrix(0, 2);
	R21 = matrix(1, 0);		R22 = matrix(1, 1);		R23 = matrix(1, 2);
	R31 = matrix(2, 0);		R32 = matrix(2, 1);		R33 = matrix(2, 2);

	double alfa = 0.0, beta = 0.0, gama = 0.0;

	if (rotSeq == E_ROTATION_SEQUENCE::E_SEQ_XYZ)
	{
		beta = std::atan2(-R31, std::sqrt(R11 * R11 + R21 * R21));

		if (std::abs(beta - M_PI / 2) < 1e-6)
		{
			beta = M_PI / 2;
			alfa = 0.0;
			gama = std::atan2(R12, R22);
		}
		else if (std::abs(beta + M_PI / 2) < 1e-6)
		{
			beta = -M_PI / 2;
			alfa = 0.0;
			gama = -std::atan2(R12, R22);
		}
		else
		{
			alfa = std::atan2(R21 / std::cos(beta), R11 / std::cos(beta));
			gama = std::atan2(R32 / std::cos(beta), R33 / std::cos(beta));
		}

		Eigen::Vector3d rotDegree = { gama, beta,alfa };
		return rotDegree * 180 / M_PI;
	}
	else if (rotSeq == E_ROTATION_SEQUENCE::E_SEQ_ZYX)
	{
		beta = std::atan2(R13, std::sqrt(R11 * R11 + R12 * R12));

		if (std::abs(beta - M_PI / 2) < 1e-6)
		{
			beta = M_PI / 2;
			alfa = 0.0;
			gama = std::atan2(R32, R22);
		}
		else if (std::abs(beta + M_PI / 2) < 1e-6)
		{
			beta = -M_PI / 2;
			alfa = 0.0;
			gama = -std::atan2(R32, R22);
		}
		else
		{
			alfa = std::atan2(-R23 / std::cos(beta), R33 / std::cos(beta));
			gama = std::atan2(-R12 / std::cos(beta), R11 / std::cos(beta));
		}

		Eigen::Vector3d rotDegree = { alfa, beta, gama };
		return rotDegree * 180 / M_PI;
	}
	else
	{
		return Eigen::Vector3d();
	}
}

Eigen::Vector3d CTransformation::rotMatrix2RotRadian(const Eigen::Matrix3d& matrix, const E_ROTATION_SEQUENCE& rotSeq)
{
	double R11, R12, R13, R21, R22, R23, R31, R32, R33;
	R11 = matrix(0, 0);		R12 = matrix(0, 1);		R13 = matrix(0, 2);
	R21 = matrix(1, 0);		R22 = matrix(1, 1);		R23 = matrix(1, 2);
	R31 = matrix(2, 0);		R32 = matrix(2, 1);		R33 = matrix(2, 2);

	double alfa = 0.0, beta = 0.0, gama = 0.0;

	if (rotSeq == E_ROTATION_SEQUENCE::E_SEQ_XYZ)
	{
		beta = std::atan2(-R31, std::sqrt(R11 * R11 + R21 * R21));

		if (std::abs(beta - M_PI / 2) < 1e-6)
		{
			beta = M_PI / 2;
			alfa = 0.0;
			gama = std::atan2(R12, R22);
		}
		else if (std::abs(beta + M_PI / 2) < 1e-6)
		{
			beta = -M_PI / 2;
			alfa = 0.0;
			gama = -std::atan2(R12, R22);
		}
		else
		{
			alfa = std::atan2(R21 / std::cos(beta), R11 / std::cos(beta));
			gama = std::atan2(R32 / std::cos(beta), R33 / std::cos(beta));
		}

		Eigen::Vector3d rotDegree = { gama, beta,alfa };
		return rotDegree;
	}
	else if (rotSeq == E_ROTATION_SEQUENCE::E_SEQ_ZYX)	// 未考虑到R11、R13为0时，解存在退化的问题
	{
		beta = std::atan2(R13, std::sqrt(R11 * R11 + R12 * R12));

		if (std::abs(beta - M_PI / 2) < 1e-6)
		{
			beta = M_PI / 2;
			alfa = 0.0;
			gama = std::atan2(R32, R22);
		}
		else if (std::abs(beta + M_PI / 2) < 1e-6)
		{
			beta = -M_PI / 2;
			alfa = 0.0;
			gama = -std::atan2(R32, R22);
		}
		else
		{
			alfa = std::atan2(-R23 / std::cos(beta), R33 / std::cos(beta));
			gama = std::atan2(-R12 / std::cos(beta), R11 / std::cos(beta));
		}

		Eigen::Vector3d rotDegree = { alfa, beta, gama };
		return rotDegree;
	}
	else
	{
		return Eigen::Vector3d();
	}
}

Eigen::Matrix4d CTransformation::pose2HmMatrix(double x, double y, double z, double rx, double ry, double rz, const E_ROTATION_SEQUENCE& rotSeq, const E_ANGLE_TYPE& angleType)
{
	Eigen::Matrix4d hmMatrix = Eigen::Matrix4d::Identity();

	Eigen::Matrix3d rotMatrix;

	if (angleType == E_ANGLE_TYPE::E_TYPE_DEGREE)
	{
		rotMatrix = rotDegree2Matrix(rx, ry, rz, rotSeq);
	}
	else
	{
		rotMatrix = rotRadian2Matrix(rx, ry, rz, rotSeq);
	}

	hmMatrix.block<3, 3>(0, 0) = rotMatrix;

	hmMatrix(0, 3) = x;
	hmMatrix(1, 3) = y;
	hmMatrix(2, 3) = z;

	return hmMatrix;
}

Eigen::Matrix4d CTransformation::pose2HmMatrix(const S_POSE& pose, const E_ROTATION_SEQUENCE& rotSeq, const E_ANGLE_TYPE& angleType)
{
	return pose2HmMatrix(pose.X, pose.Y, pose.Z, pose.Rx, pose.Ry, pose.Rz, rotSeq, angleType);
}

S_POSE CTransformation::hmMatrix2Pose(const Eigen::Matrix4d& hmMatrix, const E_ROTATION_SEQUENCE& rotSeq, const E_ANGLE_TYPE& angleType)
{
	Eigen::Matrix3d rotMatrix = hmMatrix.block<3, 3>(0, 0);

	Eigen::Vector3d rotAnagles;

	if (angleType == E_ANGLE_TYPE::E_TYPE_DEGREE)
	{
		rotAnagles = rotMatrix2RotDegree(rotMatrix, rotSeq);
	}
	else
	{
		rotAnagles = rotMatrix2RotRadian(rotMatrix, rotSeq);
	}

	double x = hmMatrix(0, 3);
	double y = hmMatrix(1, 3);
	double z = hmMatrix(2, 3);
	double rx = rotAnagles[0];
	double ry = rotAnagles[1];
	double rz = rotAnagles[2];

	return S_POSE(x, y, z, rx, ry, rz);
}

