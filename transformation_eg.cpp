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

double CTransformation_EG::degree2Radian(const double& degree)
{
	return degree * M_PI / 180;
}

double CTransformation_EG::radian2Degree(const double& radian)
{
	return radian * 180 / M_PI;
}

Eigen::Matrix3d CTransformation_EG::rotDegree2Matrix(double rx, double ry, double rz, const E_ROTATION_SEQUENCE& rotSeq)
{
	rx = degree2Radian(rx);
	ry = degree2Radian(ry);
	rz = degree2Radian(rz);

	return CTransformation_EG::rotRadian2Matrix(rx, ry, rz, rotSeq);
}

Eigen::Matrix3d CTransformation_EG::rotRadian2Matrix(double rx, double ry, double rz, const E_ROTATION_SEQUENCE& rotSeq)
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

Eigen::Vector3d CTransformation_EG::rotMatrix2RotDegree(const Eigen::Matrix3d& matrix, const E_ROTATION_SEQUENCE& rotSeq)
{
	return rotMatrix2RotRadian(matrix, rotSeq) * 180 / M_PI;
}

Eigen::Vector3d CTransformation_EG::rotMatrix2RotRadian(const Eigen::Matrix3d& matrix, const E_ROTATION_SEQUENCE& rotSeq)
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

Eigen::Matrix4d CTransformation_EG::pose2HmMatrix(double x, double y, double z, double rx, double ry, double rz, const E_ROTATION_SEQUENCE& rotSeq, const E_ANGLE_TYPE& angleType)
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

Eigen::Matrix4d CTransformation_EG::pose2HmMatrix(const S_POSE& pose, const E_ROTATION_SEQUENCE& rotSeq, const E_ANGLE_TYPE& angleType)
{
	return pose2HmMatrix(pose.X, pose.Y, pose.Z, pose.Rx, pose.Ry, pose.Rz, rotSeq, angleType);
}

S_POSE CTransformation_EG::hmMatrix2Pose(const Eigen::Matrix4d& hmMatrix, const E_ROTATION_SEQUENCE& rotSeq, const E_ANGLE_TYPE& angleType)
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

Eigen::Matrix3d CTransformation_EG::rotRadian2Matrix_Rodrigues(const double k1, const double k2, const double k3)
{
	Eigen::Vector3d rotation_vector(k1, k2, k3);		// 旋转向量
	double theta = rotation_vector.norm();				// 旋转角度

	// 角度接近0，则返回单位阵
	if (theta < 1e-10)
	{
		return Eigen::Matrix3d::Identity();
	}

	// 旋转向量 归一化
	Eigen::Vector3d k = rotation_vector / theta;

	// 反对称矩阵 [k]_x
	Eigen::Matrix3d K;
	K << 0, -k.z(), k.y(),
		k.z(), 0, -k.x(),
		-k.y(), k.x(), 0;

	// Rodrigues 公式 R=I+sin(θ)[K]_x+(1-cos(θ))[K]_x^2
	Eigen::Matrix3d R = Eigen::Matrix3d::Identity()
		+ std::sin(theta) * K
		+ (1 - std::cos(theta)) * (K * K);

	return R;
}

Eigen::Matrix3d CTransformation_EG::rotDegree2Matrix_Rodrigues(const double k1, const double k2, const double k3)
{
	return rotRadian2Matrix_Rodrigues(degree2Radian(k1), degree2Radian(k2), degree2Radian(k3));
}

Eigen::Vector3d CTransformation_EG::rotMatrix2RotRadian_Rodrigues(const Eigen::Matrix3d& rotMat)
{
	// 检查旋转矩阵是否为正交矩阵（R * R^T = I）且行列式为 1
	if (!Eigen::Matrix3d::Identity().isApprox(rotMat.transpose() * rotMat, 1e-10) || std::abs(rotMat.determinant() - 1.0) > 1e-10)
	{
		return Eigen::Vector3d(0, 0, 0);
	}

	// 计算旋转角度 θ
	double theta = std::acos((rotMat.trace() - 1.0) / 2.0);

	// 如果 θ 接近 0，说明旋转矩阵接近单位矩阵
	if (std::abs(theta) < 1e-10)
	{
		return Eigen::Vector3d(0, 0, 0);
	}

	// 如果 θ 接近 π，需特别处理以避免数值不稳定性
	if (std::abs(theta - M_PI) < 1e-10)
	{
		// 找到旋转轴方向（从反对称[K]_x提取）
		Eigen::Matrix3d R_plus_I = rotMat + Eigen::Matrix3d::Identity();
		Eigen::Vector3d k;
		if (R_plus_I.col(0).norm() > 1e-10)
		{
			k = R_plus_I.col(0).normalized();
		}
		else if (R_plus_I.col(1).norm() > 1e-10)
		{
			k = R_plus_I.col(1).normalized();
		}
		else
		{
			k = R_plus_I.col(2).normalized();
		}

		return theta * k;
	}

	// 计算旋转轴 k
	Eigen::Vector3d k;
	k.x() = rotMat(2, 1) - rotMat(1, 2);
	k.y() = rotMat(0, 2) - rotMat(2, 0);
	k.z() = rotMat(1, 0) - rotMat(0, 1);
	k = k.normalized();

	return theta * k;
}

Eigen::Vector3d CTransformation_EG::rotMatrix2RotDegree_Rodrigues(const Eigen::Matrix3d& rotMat)
{
	return rotMatrix2RotRadian_Rodrigues(rotMat) * 180 / M_PI;
}

Eigen::Matrix4d CTransformation_EG::pose2HmMatrix_Rodrigues(const double x, const double y, const double z, const double k1, const double k2, const double k3, const E_ANGLE_TYPE& angleType)
{
	Eigen::Matrix4d hmMatrix = Eigen::Matrix4d::Identity();

	Eigen::Matrix3d rotMatrix;

	if (angleType == E_ANGLE_TYPE::E_TYPE_DEGREE)
	{
		rotMatrix = rotDegree2Matrix_Rodrigues(k1, k2, k3);
	}
	else
	{
		rotMatrix = rotRadian2Matrix_Rodrigues(k1, k2, k3);
	}

	hmMatrix.block<3, 3>(0, 0) = rotMatrix;

	hmMatrix(0, 3) = x;
	hmMatrix(1, 3) = y;
	hmMatrix(2, 3) = z;

	return hmMatrix;
}

Eigen::Matrix4d CTransformation_EG::pose2HmMatrix_Rodrigues(const S_POSE& pose, const E_ANGLE_TYPE& angleType)
{
	return pose2HmMatrix_Rodrigues(pose.X, pose.Y, pose.Z, pose.Rx, pose.Ry, pose.Rz);
}

