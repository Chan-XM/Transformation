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
#include <algorithm>

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

bool CTransformation_EG::rotMatrix2RotDegree(const Eigen::Matrix3d& matrix, const E_ROTATION_SEQUENCE& rotSeq, Eigen::Vector3d& rotVec)
{
	if (!rotMatrix2RotRadian(matrix, rotSeq, rotVec))
	{
		return false;
	}

	rotVec *= 180 / M_PI;
	return true;
}

Eigen::Vector3d CTransformation_EG::rotMatrix2RotRadian(const Eigen::Matrix3d& matrix, const E_ROTATION_SEQUENCE& rotSeq)
{
	Eigen::Vector3d rotVec = Eigen::Vector3d::Zero();

	// 失败时保持返回零向量；需要区分「输入非法」与「零旋转」请使用返回 bool 的重载
	rotMatrix2RotRadian(matrix, rotSeq, rotVec);

	return rotVec;
}

bool CTransformation_EG::rotMatrix2RotRadian(const Eigen::Matrix3d& matrix, const E_ROTATION_SEQUENCE& rotSeq, Eigen::Vector3d& rotVec)
{
	rotVec = Eigen::Vector3d::Zero();

	// 欧拉角反解隐含「输入是纯旋转矩阵」这一前提：含缩放或镜像的矩阵会解出无意义的角度
	if (!isValidRotationMatrix(matrix))
	{
		return false;
	}

	double R11, R12, R13, R21, R22, R23, R31, R32, R33;
	R11 = matrix(0, 0);		R12 = matrix(0, 1);		R13 = matrix(0, 2);
	R21 = matrix(1, 0);		R22 = matrix(1, 1);		R23 = matrix(1, 2);
	R31 = matrix(2, 0);		R32 = matrix(2, 1);		R33 = matrix(2, 2);

	// 统一用 rx / ry / rz 命名，避免两个分支里变量语义互换造成误读
	double rx = 0.0, ry = 0.0, rz = 0.0;

	if (rotSeq == E_ROTATION_SEQUENCE::E_SEQ_XYZ)
	{
		ry = std::atan2(-R31, std::sqrt(R11 * R11 + R21 * R21));

		if (std::abs(ry - M_PI / 2) < TRANSFORM_TOL::BRANCH_GIMBAL_LOCK)
		{
			ry = M_PI / 2;
			rz = 0.0;
			rx = std::atan2(R12, R22);
		}
		else if (std::abs(ry + M_PI / 2) < TRANSFORM_TOL::BRANCH_GIMBAL_LOCK)
		{
			ry = -M_PI / 2;
			rz = 0.0;
			rx = -std::atan2(R12, R22);
		}
		else
		{
			rz = std::atan2(R21 / std::cos(ry), R11 / std::cos(ry));
			rx = std::atan2(R32 / std::cos(ry), R33 / std::cos(ry));
		}

		rotVec = Eigen::Vector3d(rx, ry, rz);
		return true;
	}
	else if (rotSeq == E_ROTATION_SEQUENCE::E_SEQ_ZYX)	// 未考虑到R11、R13为0时，解存在退化的问题
	{
		ry = std::atan2(R13, std::sqrt(R11 * R11 + R12 * R12));

		if (std::abs(ry - M_PI / 2) < TRANSFORM_TOL::BRANCH_GIMBAL_LOCK)
		{
			ry = M_PI / 2;
			rx = 0.0;
			rz = std::atan2(R32, R22);
		}
		else if (std::abs(ry + M_PI / 2) < TRANSFORM_TOL::BRANCH_GIMBAL_LOCK)
		{
			ry = -M_PI / 2;
			rx = 0.0;
			rz = -std::atan2(R32, R22);
		}
		else
		{
			rx = std::atan2(-R23 / std::cos(ry), R33 / std::cos(ry));
			rz = std::atan2(-R12 / std::cos(ry), R11 / std::cos(ry));
		}

		rotVec = Eigen::Vector3d(rx, ry, rz);
		return true;
	}

	// 旋转顺序非法（正常不会到达）
	return false;
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
	S_POSE pose;

	// 失败时返回各分量均为零的位姿；需要区分「矩阵非法」与「零位姿」请使用返回 bool 的重载
	hmMatrix2Pose(hmMatrix, pose, rotSeq, angleType);

	return pose;
}

bool CTransformation_EG::hmMatrix2Pose(const Eigen::Matrix4d& hmMatrix, S_POSE& pose, const E_ROTATION_SEQUENCE& rotSeq, const E_ANGLE_TYPE& angleType)
{
	pose = S_POSE();

	if (!isValidHmMatrix(hmMatrix))
	{
		return false;
	}

	const Eigen::Matrix3d rotMatrix = hmMatrix.block<3, 3>(0, 0);

	Eigen::Vector3d rotAngles;
	const bool ok = (angleType == E_ANGLE_TYPE::E_TYPE_DEGREE)
		? rotMatrix2RotDegree(rotMatrix, rotSeq, rotAngles)
		: rotMatrix2RotRadian(rotMatrix, rotSeq, rotAngles);
	if (!ok)
	{
		return false;
	}

	pose.X = hmMatrix(0, 3);
	pose.Y = hmMatrix(1, 3);
	pose.Z = hmMatrix(2, 3);
	pose.Rx = rotAngles[0];
	pose.Ry = rotAngles[1];
	pose.Rz = rotAngles[2];

	return true;
}

Eigen::Matrix3d CTransformation_EG::rotRadian2Matrix_Rodrigues(const double k1, const double k2, const double k3)
{
	Eigen::Vector3d rotation_vector(k1, k2, k3);		// 旋转向量
	double theta = rotation_vector.norm();				// 旋转角度

	// 角度接近0，则返回单位阵
	if (theta < TRANSFORM_TOL::BRANCH_ZERO_ANGLE)
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
	Eigen::Vector3d rotVec = Eigen::Vector3d::Zero();

	// 失败时保持返回零向量；需要区分「输入非法」与「零旋转」请使用返回 bool 的重载
	rotMatrix2RotRadian_Rodrigues(rotMat, rotVec);

	return rotVec;
}

bool CTransformation_EG::rotMatrix2RotRadian_Rodrigues(const Eigen::Matrix3d& rotMat, Eigen::Vector3d& rotVec)
{
	rotVec = Eigen::Vector3d::Zero();

	if (!isValidRotationMatrix(rotMat))
	{
		return false;
	}

	// 夹取到 acos 的定义域：即使通过上面的校验，(trace - 1) / 2 仍可能略微超出 [-1, 1]
	// 不夹取时 acos 会返回 NaN，且 NaN 会一路传播到调用方
	const double cosTheta = std::max(-1.0, std::min(1.0, (rotMat.trace() - 1.0) / 2.0));

	// 计算旋转角度 theta
	const double theta = std::acos(cosTheta);

	// 如果 theta 接近 0，说明旋转矩阵接近单位矩阵，旋转向量取零向量
	if (theta < TRANSFORM_TOL::BRANCH_ZERO_ANGLE)
	{
		return true;
	}

	// 如果 theta 接近 pi，需特别处理以避免数值不稳定性
	if (std::abs(theta - M_PI) < TRANSFORM_TOL::BRANCH_NEAR_PI)
	{
		// 找到旋转轴方向：theta = pi 时 R + I = 2 * k * k^T
		// 必须取范数最大的那一列：若某分量 k_i 很小，对应列会被 O(pi - theta) 的项污染，
		// 此时归一化得到的方向误差可达 1e-2 量级
		const Eigen::Matrix3d R_plus_I = rotMat + Eigen::Matrix3d::Identity();

		int maxCol = 0;
		double maxNorm = R_plus_I.col(0).norm();
		for (int c = 1; c < 3; ++c)
		{
			const double colNorm = R_plus_I.col(c).norm();
			if (colNorm > maxNorm)
			{
				maxNorm = colNorm;
				maxCol = c;
			}
		}

		if (maxNorm < TRANSFORM_TOL::BRANCH_AXIS_ELEMENT)
		{
			// 理论上不可达：合法的旋转矩阵在 theta = pi 时必有 2 * k * k^T 的非零列
			return false;
		}

		// 固定近 pi 时的轴向符号：令绝对值最大的分量为正。
		// theta = pi 时 k 与 -k 表示同一个旋转，R + I 无法区分二者；若不固定，
		// 同一个旋转在不同输入下会返回方向相反的旋转向量，破坏插值/滤波的连续性。
		Eigen::Vector3d axis = R_plus_I.col(maxCol).normalized();
		int pivot = 0;
		for (int i = 1; i < 3; ++i)
		{
			if (std::abs(axis[i]) > std::abs(axis[pivot]))
			{
				pivot = i;
			}
		}
		if (axis[pivot] < 0.0)
		{
			axis = -axis;
		}

		rotVec = theta * axis;
		return true;
	}

	// 由反对称部分提取旋转轴：R - R^T = 2 * sin(theta) * [k]_x
	Eigen::Vector3d k;
	k.x() = rotMat(2, 1) - rotMat(1, 2);
	k.y() = rotMat(0, 2) - rotMat(2, 0);
	k.z() = rotMat(1, 0) - rotMat(0, 1);

	const double kNorm = k.norm();
	if (kNorm < TRANSFORM_TOL::BRANCH_ZERO_ANGLE)
	{
		// 正交矩阵不应进入此分支（theta 非零时 sin(theta) 必非零），仅作防御性判断
		return false;
	}

	rotVec = theta * (k / kNorm);
	return true;
}

Eigen::Vector3d CTransformation_EG::rotMatrix2RotDegree_Rodrigues(const Eigen::Matrix3d& rotMat)
{
	return rotMatrix2RotRadian_Rodrigues(rotMat) * 180 / M_PI;
}

bool CTransformation_EG::rotMatrix2RotDegree_Rodrigues(const Eigen::Matrix3d& rotMat, Eigen::Vector3d& rotVec)
{
	if (!rotMatrix2RotRadian_Rodrigues(rotMat, rotVec))
	{
		return false;
	}

	rotVec *= 180 / M_PI;
	return true;
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
	return pose2HmMatrix_Rodrigues(pose.X, pose.Y, pose.Z, pose.Rx, pose.Ry, pose.Rz, angleType);
}

// ---------------------------------------------------------------------------
// 私有辅助：输入校验
// ---------------------------------------------------------------------------
bool CTransformation_EG::isValidRotationMatrix(const Eigen::Matrix3d& rotMatrix)
{
	// 正交性：R^T * R = I（isApprox 使用相对容差）
	if (!Eigen::Matrix3d::Identity().isApprox(rotMatrix.transpose() * rotMatrix, TRANSFORM_TOL::CHECK_ORTHOGONAL))
	{
		return false;
	}

	// 行列式为 1：用于排除镜像矩阵（det = -1）
	return std::abs(rotMatrix.determinant() - 1.0) <= TRANSFORM_TOL::CHECK_ORTHOGONAL;
}

bool CTransformation_EG::isValidHmMatrix(const Eigen::Matrix4d& hmMatrix)
{
	// 最后一行为 [0 0 0 1]
	const Eigen::Vector4d lastRow = hmMatrix.row(3).transpose();
	const Eigen::Vector4d expectedLastRow(0.0, 0.0, 0.0, 1.0);
	if ((lastRow - expectedLastRow).norm() > TRANSFORM_TOL::CHECK_ORTHOGONAL)
	{
		return false;
	}

	return isValidRotationMatrix(hmMatrix.block<3, 3>(0, 0));
}

// ---------------------------------------------------------------------------
// 齐次矩阵求逆 / 位姿求逆
// ---------------------------------------------------------------------------
bool CTransformation_EG::invHmMatrix(const Eigen::Matrix4d& hmMatrix, Eigen::Matrix4d& invMatrix)
{
	invMatrix = Eigen::Matrix4d::Identity();

	if (!isValidHmMatrix(hmMatrix))
	{
		return false;
	}

	// 刚体变换的逆有解析解：R^-1 = R^T，t^-1 = -R^T * t
	// 相比通用矩阵求逆更快，也不会因条件数恶化而失真
	const Eigen::Matrix3d rotTranspose = hmMatrix.block<3, 3>(0, 0).transpose();
	invMatrix.block<3, 3>(0, 0) = rotTranspose;
	invMatrix.block<3, 1>(0, 3) = -rotTranspose * hmMatrix.block<3, 1>(0, 3);

	return true;
}

bool CTransformation_EG::invPose(const S_POSE& pose, S_POSE& outPose, const E_ROTATION_SEQUENCE& rotSeq, const E_ANGLE_TYPE& angleType)
{
	outPose = S_POSE();

	Eigen::Matrix4d invMatrix;
	if (!invHmMatrix(pose2HmMatrix(pose, rotSeq, angleType), invMatrix))
	{
		return false;
	}

	return hmMatrix2Pose(invMatrix, outPose, rotSeq, angleType);
}

// ---------------------------------------------------------------------------
// 齐次矩阵复合 / 位姿复合
// ---------------------------------------------------------------------------
bool CTransformation_EG::composeHmMatrix(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b, Eigen::Matrix4d& outMatrix)
{
	outMatrix = Eigen::Matrix4d::Identity();

	if (!isValidHmMatrix(a) || !isValidHmMatrix(b))
	{
		return false;
	}

	outMatrix = a * b;
	return true;
}

bool CTransformation_EG::composePose(const S_POSE& a, const S_POSE& b, S_POSE& outPose, const E_ROTATION_SEQUENCE& rotSeq, const E_ANGLE_TYPE& angleType)
{
	outPose = S_POSE();

	Eigen::Matrix4d composedMatrix;
	if (!composeHmMatrix(pose2HmMatrix(a, rotSeq, angleType), pose2HmMatrix(b, rotSeq, angleType), composedMatrix))
	{
		return false;
	}

	return hmMatrix2Pose(composedMatrix, outPose, rotSeq, angleType);
}

// ---------------------------------------------------------------------------
// 位姿插值
// ---------------------------------------------------------------------------
bool CTransformation_EG::interpolatePose(const S_POSE& from, const S_POSE& to, double t, S_POSE& outPose, const E_ROTATION_SEQUENCE& rotSeq, const E_ANGLE_TYPE& angleType)
{
	outPose = S_POSE();

	const Eigen::Matrix3d fromRot = (angleType == E_ANGLE_TYPE::E_TYPE_DEGREE)
		? rotDegree2Matrix(from.Rx, from.Ry, from.Rz, rotSeq)
		: rotRadian2Matrix(from.Rx, from.Ry, from.Rz, rotSeq);
	const Eigen::Matrix3d toRot = (angleType == E_ANGLE_TYPE::E_TYPE_DEGREE)
		? rotDegree2Matrix(to.Rx, to.Ry, to.Rz, rotSeq)
		: rotRadian2Matrix(to.Rx, to.Ry, to.Rz, rotSeq);

	// 旋转沿测地线插值：相对旋转 R_from^T * R_to 的轴角表示保证转角落在 [0, pi]，
	// 因此 t 在 [0,1] 内走最短路径；t 超出 [0,1] 时自然退化为外插。
	// 注意：相对转角恰为 pi 时轴向符号本身不唯一，插值路径由 Eigen 的轴角实现决定。
	const Eigen::AngleAxisd relRot(fromRot.transpose() * toRot);
	const Eigen::Matrix3d rotMatrix = fromRot * Eigen::AngleAxisd(t * relRot.angle(), relRot.axis()).toRotationMatrix();

	// 平移按线性插值
	const Eigen::Vector3d position = (1.0 - t) * Eigen::Vector3d(from.X, from.Y, from.Z)
		+ t * Eigen::Vector3d(to.X, to.Y, to.Z);

	Eigen::Matrix4d hmMatrix = Eigen::Matrix4d::Identity();
	hmMatrix.block<3, 3>(0, 0) = rotMatrix;
	hmMatrix.block<3, 1>(0, 3) = position;

	return hmMatrix2Pose(hmMatrix, outPose, rotSeq, angleType);
}