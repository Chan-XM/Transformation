/*****************************************************************************
*  @file     transformation_cv.cpp                                           *
*  @brief    用OpenCV实现三维空间中位姿与矩阵之间的变换                          *
*                                                                            *
*  @author   Chan_XM                                                         *
*  @email    chan_xm14@163.com			                                     *
*  @version  V1.0														     *
*  @date     2024/11/16													     *
*  @license  NON															 *
*****************************************************************************/
#include "transformation_cv.h"
#include <iostream>

double CTransformation_CV::degree2Radian(const double& degree)
{
	return degree * M_PI / 180.0;
}

double CTransformation_CV::radian2Degree(const double& radian)
{
	return radian * 180.0 / M_PI;
}

cv::Mat CTransformation_CV::rotDegree2Matrix(double rx, double ry, double rz, const E_ROTATION_SEQUENCE& rotSeq)
{
    rx = degree2Radian(rx);
    ry = degree2Radian(ry);
    rz = degree2Radian(rz);

    return rotRadian2Matrix(rx, ry, rz, rotSeq);
}

cv::Mat CTransformation_CV::rotRadian2Matrix(double rx, double ry, double rz, const E_ROTATION_SEQUENCE& rotSeq)
{
    cv::Mat rotX = (cv::Mat_<double>(3, 3) <<
        1, 0, 0,
        0, cos(rx), -sin(rx),
        0, sin(rx), cos(rx));

    cv::Mat rotY = (cv::Mat_<double>(3, 3) <<
        cos(ry), 0, sin(ry),
        0, 1, 0,
        -sin(ry), 0, cos(ry));

    cv::Mat rotZ = (cv::Mat_<double>(3, 3) <<
        cos(rz), -sin(rz), 0,
        sin(rz), cos(rz), 0,
        0, 0, 1);

    cv::Mat rotationMatrix;
    if (rotSeq == E_ROTATION_SEQUENCE::E_SEQ_XYZ) {
        rotationMatrix = rotZ * rotY * rotX;
    }
    else {
        rotationMatrix = rotX * rotY * rotZ;
    }

    return rotationMatrix;
}

cv::Vec3d CTransformation_CV::rotMatrix2RotDegree(const cv::Mat& matrix, const E_ROTATION_SEQUENCE& rotSeq)
{
    return rotMatrix2RotRadian(matrix, rotSeq) * 180 / M_PI;
}

bool CTransformation_CV::rotMatrix2RotDegree(const cv::Mat& matrix, const E_ROTATION_SEQUENCE& rotSeq, cv::Vec3d& rotVec)
{
    if (!rotMatrix2RotRadian(matrix, rotSeq, rotVec))
    {
        return false;
    }

    rotVec = rotVec * 180 / M_PI;
    return true;
}

cv::Vec3d CTransformation_CV::rotMatrix2RotRadian(const cv::Mat& matrix, const E_ROTATION_SEQUENCE& rotSeq)
{
    cv::Vec3d rotVec(0.0, 0.0, 0.0);

    // 失败时保持返回零向量；需要区分「输入非法」与「零旋转」请使用返回 bool 的重载
    rotMatrix2RotRadian(matrix, rotSeq, rotVec);

    return rotVec;
}

bool CTransformation_CV::rotMatrix2RotRadian(const cv::Mat& matrix, const E_ROTATION_SEQUENCE& rotSeq, cv::Vec3d& rotVec)
{
    rotVec = cv::Vec3d(0.0, 0.0, 0.0);

    cv::Mat matrix64;
    if (!toRotMatrix64(matrix, matrix64))
    {
        return false;
    }

    // 欧拉角反解隐含「输入是纯旋转矩阵」这一前提：含缩放或镜像的矩阵会解出无意义的角度
    if (!isValidRotationMatrix64(matrix64))
    {
        return false;
    }

    const double R11 = matrix64.at<double>(0, 0);
    const double R12 = matrix64.at<double>(0, 1);
    const double R13 = matrix64.at<double>(0, 2);
    const double R21 = matrix64.at<double>(1, 0);
    const double R22 = matrix64.at<double>(1, 1);
    const double R23 = matrix64.at<double>(1, 2);
    const double R31 = matrix64.at<double>(2, 0);
    const double R32 = matrix64.at<double>(2, 1);
    const double R33 = matrix64.at<double>(2, 2);

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

        rotVec = cv::Vec3d(rx, ry, rz);
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

        rotVec = cv::Vec3d(rx, ry, rz);
        return true;
    }

    // 旋转顺序非法（正常不会到达）
    return false;
}

cv::Mat CTransformation_CV::pose2HmMatrix(double x, double y, double z, double rx, double ry, double rz, const E_ROTATION_SEQUENCE& rotSeq, const E_ANGLE_TYPE& angleType)
{
    cv::Mat hmMatrix = cv::Mat::eye(4, 4, CV_64F);

    cv::Mat rotMatrix;

    if (angleType == E_ANGLE_TYPE::E_TYPE_DEGREE)
    {
        rotMatrix = rotDegree2Matrix(rx, ry, rz, rotSeq);
    }
    else
    {
        rotMatrix = rotRadian2Matrix(rx, ry, rz, rotSeq);
    }

    rotMatrix.copyTo(hmMatrix(cv::Rect(0, 0, 3, 3)));

    hmMatrix.at<double>(0, 3) = x;
    hmMatrix.at<double>(1, 3) = y;
    hmMatrix.at<double>(2, 3) = z;

    return hmMatrix;
}

cv::Mat CTransformation_CV::pose2HmMatrix(const S_POSE& pose, const E_ROTATION_SEQUENCE& rotSeq, const E_ANGLE_TYPE& angleType)
{
    return pose2HmMatrix(pose.X, pose.Y, pose.Z, pose.Rx, pose.Ry, pose.Rz, rotSeq, angleType);
}

S_POSE CTransformation_CV::hmMatrix2Pose(const cv::Mat& hmMatrix, const E_ROTATION_SEQUENCE& rotSeq, const E_ANGLE_TYPE& angleType)
{
    S_POSE pose;

    // 失败时返回各分量均为零的位姿；需要区分「矩阵非法」与「零位姿」请使用返回 bool 的重载
    hmMatrix2Pose(hmMatrix, pose, rotSeq, angleType);

    return pose;
}

bool CTransformation_CV::hmMatrix2Pose(const cv::Mat& hmMatrix, S_POSE& pose, const E_ROTATION_SEQUENCE& rotSeq, const E_ANGLE_TYPE& angleType)
{
    pose = S_POSE();

    cv::Mat hmMatrix64;
    if (!toHmMatrix64(hmMatrix, hmMatrix64))
    {
        return false;
    }

    if (!isValidHmMatrix64(hmMatrix64))
    {
        return false;
    }

    const cv::Mat rotMatrix64 = hmMatrix64(cv::Rect(0, 0, 3, 3)).clone();

    cv::Vec3d rotAngles;
    const bool ok = (angleType == E_ANGLE_TYPE::E_TYPE_DEGREE)
        ? rotMatrix2RotDegree(rotMatrix64, rotSeq, rotAngles)
        : rotMatrix2RotRadian(rotMatrix64, rotSeq, rotAngles);
    if (!ok)
    {
        return false;
    }

    pose.X = hmMatrix64.at<double>(0, 3);
    pose.Y = hmMatrix64.at<double>(1, 3);
    pose.Z = hmMatrix64.at<double>(2, 3);
    pose.Rx = rotAngles[0];
    pose.Ry = rotAngles[1];
    pose.Rz = rotAngles[2];

    return true;
}

cv::Mat CTransformation_CV::rotRadian2Matrix_Rodrigues(const double k1, const double k2, const double k3)
{
    cv::Mat vector = (cv::Mat_<double>(3, 1) << k1, k2, k3);
    cv::Mat rotMat;
    cv::Rodrigues(vector, rotMat);

    return rotMat;
}

cv::Mat CTransformation_CV::rotDegree2Matrix_Rodrigues(const double k1, const double k2, const double k3)
{
    return rotRadian2Matrix_Rodrigues(degree2Radian(k1), degree2Radian(k2), degree2Radian(k3));
}

cv::Vec3d CTransformation_CV::rotMatrix2RotRadian_Rodrigues(const cv::Mat& rotMat)
{
    cv::Vec3d rotVec(0.0, 0.0, 0.0);

    // 失败时保持返回零向量；需要区分「输入非法」与「零旋转」请使用返回 bool 的重载
    rotMatrix2RotRadian_Rodrigues(rotMat, rotVec);

    return rotVec;
}

bool CTransformation_CV::rotMatrix2RotRadian_Rodrigues(const cv::Mat& rotMat, cv::Vec3d& rotVec)
{
    rotVec = cv::Vec3d(0.0, 0.0, 0.0);

    // 形状/通道/深度校验，并统一转成 CV_64F（cv::Rodrigues 的输出深度跟随输入）
    cv::Mat rotMat64;
    if (!toRotMatrix64(rotMat, rotMat64))
    {
        return false;
    }

    if (!isValidRotationMatrix64(rotMat64))
    {
        return false;
    }

    cv::Mat rotVecMat;
    cv::Rodrigues(rotMat64, rotVecMat);

    rotVec[0] = rotVecMat.at<double>(0);
    rotVec[1] = rotVecMat.at<double>(1);
    rotVec[2] = rotVecMat.at<double>(2);

    // 与 Eigen 侧保持一致：近 pi 时固定轴向符号
    canonicalizeNearPiAxis(rotVec);

    return true;
}

cv::Vec3d CTransformation_CV::rotMatrix2RotDegree_Rodrigues(const cv::Mat& rotMat)
{
    cv::Vec3d rotVec = rotMatrix2RotRadian_Rodrigues(rotMat);

    rotVec = rotVec * 180 / M_PI;

    return rotVec;
}

bool CTransformation_CV::rotMatrix2RotDegree_Rodrigues(const cv::Mat& rotMat, cv::Vec3d& rotVec)
{
    if (!rotMatrix2RotRadian_Rodrigues(rotMat, rotVec))
    {
        return false;
    }

    rotVec = rotVec * 180 / M_PI;
    return true;
}

cv::Mat CTransformation_CV::pose2HmMatrix_Rodrigues(const double x, const double y, const double z, const double k1, const double k2, const double k3, const E_ANGLE_TYPE& angleType)
{
    cv::Mat hmMatrix = cv::Mat::eye(4, 4, CV_64F);

    cv::Mat rotMatrix;

    if (angleType == E_ANGLE_TYPE::E_TYPE_DEGREE)
    {
        rotMatrix = rotDegree2Matrix_Rodrigues(k1, k2, k3);
    }
    else
    {
        rotMatrix = rotRadian2Matrix_Rodrigues(k1, k2, k3);
    }

    rotMatrix.copyTo(hmMatrix(cv::Rect(0, 0, 3, 3)));

    hmMatrix.at<double>(0, 3) = x;
    hmMatrix.at<double>(1, 3) = y;
    hmMatrix.at<double>(2, 3) = z;

    return hmMatrix;
}

cv::Mat CTransformation_CV::pose2HmMatrix_Rodrigues(const S_POSE& pose, const E_ANGLE_TYPE& angleType)
{
    return pose2HmMatrix_Rodrigues(pose.X, pose.Y, pose.Z, pose.Rx, pose.Ry, pose.Rz, angleType);
}

// ---------------------------------------------------------------------------
// 私有辅助：形状/深度校验与深度统一
// ---------------------------------------------------------------------------
bool CTransformation_CV::toRotMatrix64(const cv::Mat& rotMatrix, cv::Mat& rotMatrix64)
{
    if (rotMatrix.cols != 3 || rotMatrix.rows != 3)
    {
        std::cerr << "rotation matrix != 3x3" << std::endl;
        return false;
    }

    if (rotMatrix.channels() != 1)
    {
        std::cerr << "rotation matrix channels != 1" << std::endl;
        return false;
    }

    if (CV_MAT_DEPTH(rotMatrix.type()) != CV_32F && CV_MAT_DEPTH(rotMatrix.type()) != CV_64F)
    {
        std::cerr << "rotation matrix type != float/double" << std::endl;
        return false;
    }

    // 统一转成 CV_64F：cv::Rodrigues 等函数输出深度跟随输入，混用会让 at<T> 读错数据
    if (rotMatrix.type() == CV_64F)
    {
        rotMatrix64 = rotMatrix;
    }
    else
    {
        rotMatrix.convertTo(rotMatrix64, CV_64F);
    }

    return true;
}

bool CTransformation_CV::toHmMatrix64(const cv::Mat& hmMatrix, cv::Mat& hmMatrix64)
{
    if (hmMatrix.cols != 4 || hmMatrix.rows != 4)
    {
        std::cerr << "homogeneous matrix != 4x4" << std::endl;
        return false;
    }

    if (hmMatrix.channels() != 1)
    {
        std::cerr << "homogeneous matrix channels != 1" << std::endl;
        return false;
    }

    if (CV_MAT_DEPTH(hmMatrix.type()) != CV_32F && CV_MAT_DEPTH(hmMatrix.type()) != CV_64F)
    {
        std::cerr << "homogeneous matrix type != float/double" << std::endl;
        return false;
    }

    if (hmMatrix.type() == CV_64F)
    {
        hmMatrix64 = hmMatrix;
    }
    else
    {
        hmMatrix.convertTo(hmMatrix64, CV_64F);
    }

    return true;
}

bool CTransformation_CV::isValidRotationMatrix64(const cv::Mat& rotMatrix64)
{
    // 正交性：R^T * R = I（Frobenius 范数）
    if (cv::norm(rotMatrix64.t() * rotMatrix64 - cv::Mat::eye(3, 3, CV_64F)) > TRANSFORM_TOL::CHECK_ORTHOGONAL)
    {
        return false;
    }

    // 行列式为 1：用于排除镜像矩阵（det = -1）
    return std::abs(cv::determinant(rotMatrix64) - 1.0) <= TRANSFORM_TOL::CHECK_ORTHOGONAL;
}

bool CTransformation_CV::isValidHmMatrix64(const cv::Mat& hmMatrix64)
{
    // 最后一行为 [0 0 0 1]
    if (std::abs(hmMatrix64.at<double>(3, 0)) > TRANSFORM_TOL::CHECK_ORTHOGONAL
        || std::abs(hmMatrix64.at<double>(3, 1)) > TRANSFORM_TOL::CHECK_ORTHOGONAL
        || std::abs(hmMatrix64.at<double>(3, 2)) > TRANSFORM_TOL::CHECK_ORTHOGONAL
        || std::abs(hmMatrix64.at<double>(3, 3) - 1.0) > TRANSFORM_TOL::CHECK_ORTHOGONAL)
    {
        return false;
    }

    return isValidRotationMatrix64(hmMatrix64(cv::Rect(0, 0, 3, 3)).clone());
}

void CTransformation_CV::canonicalizeNearPiAxis(cv::Vec3d& rotVec)
{
    const double theta = cv::norm(rotVec);
    if (std::abs(theta - M_PI) >= TRANSFORM_TOL::BRANCH_NEAR_PI)
    {
        return;
    }

    // theta = pi 时 k 与 -k 表示同一个旋转；固定为「绝对值最大的分量为正」，
    // 与 Eigen 侧约定一致，避免同一旋转返回方向相反的旋转向量
    int pivot = 0;
    for (int i = 1; i < 3; ++i)
    {
        if (std::abs(rotVec[i]) > std::abs(rotVec[pivot]))
        {
            pivot = i;
        }
    }

    if (rotVec[pivot] < 0.0)
    {
        rotVec = cv::Vec3d(-rotVec[0], -rotVec[1], -rotVec[2]);
    }
}

// ---------------------------------------------------------------------------
// 齐次矩阵求逆 / 位姿求逆
// ---------------------------------------------------------------------------
bool CTransformation_CV::invHmMatrix(const cv::Mat& hmMatrix, cv::Mat& invMatrix)
{
    invMatrix = cv::Mat::eye(4, 4, CV_64F);

    cv::Mat hmMatrix64;
    if (!toHmMatrix64(hmMatrix, hmMatrix64))
    {
        return false;
    }

    if (!isValidHmMatrix64(hmMatrix64))
    {
        return false;
    }

    // 刚体变换的逆有解析解：R^-1 = R^T，t^-1 = -R^T * t
    const cv::Mat rotMatrix64 = hmMatrix64(cv::Rect(0, 0, 3, 3)).clone();
    const cv::Mat rotTranspose = rotMatrix64.t();
    rotTranspose.copyTo(invMatrix(cv::Rect(0, 0, 3, 3)));

    const cv::Mat translation = hmMatrix64(cv::Rect(3, 0, 1, 3)).clone();
    const cv::Mat invTranslation = -rotTranspose * translation;
    invTranslation.copyTo(invMatrix(cv::Rect(3, 0, 1, 3)));

    return true;
}

bool CTransformation_CV::invPose(const S_POSE& pose, S_POSE& outPose, const E_ROTATION_SEQUENCE& rotSeq, const E_ANGLE_TYPE& angleType)
{
    outPose = S_POSE();

    cv::Mat invMatrix;
    if (!invHmMatrix(pose2HmMatrix(pose, rotSeq, angleType), invMatrix))
    {
        return false;
    }

    return hmMatrix2Pose(invMatrix, outPose, rotSeq, angleType);
}

// ---------------------------------------------------------------------------
// 齐次矩阵复合 / 位姿复合
// ---------------------------------------------------------------------------
bool CTransformation_CV::composeHmMatrix(const cv::Mat& a, const cv::Mat& b, cv::Mat& outMatrix)
{
    outMatrix = cv::Mat::eye(4, 4, CV_64F);

    cv::Mat a64;
    if (!toHmMatrix64(a, a64))
    {
        return false;
    }

    cv::Mat b64;
    if (!toHmMatrix64(b, b64))
    {
        return false;
    }

    if (!isValidHmMatrix64(a64) || !isValidHmMatrix64(b64))
    {
        return false;
    }

    outMatrix = a64 * b64;
    return true;
}

bool CTransformation_CV::composePose(const S_POSE& a, const S_POSE& b, S_POSE& outPose, const E_ROTATION_SEQUENCE& rotSeq, const E_ANGLE_TYPE& angleType)
{
    outPose = S_POSE();

    cv::Mat composedMatrix;
    if (!composeHmMatrix(pose2HmMatrix(a, rotSeq, angleType), pose2HmMatrix(b, rotSeq, angleType), composedMatrix))
    {
        return false;
    }

    return hmMatrix2Pose(composedMatrix, outPose, rotSeq, angleType);
}

// ---------------------------------------------------------------------------
// 位姿插值
// ---------------------------------------------------------------------------
bool CTransformation_CV::interpolatePose(const S_POSE& from, const S_POSE& to, double t, S_POSE& outPose, const E_ROTATION_SEQUENCE& rotSeq, const E_ANGLE_TYPE& angleType)
{
    outPose = S_POSE();

    const bool isDegree = (angleType == E_ANGLE_TYPE::E_TYPE_DEGREE);
    const cv::Mat fromRot = isDegree
        ? rotDegree2Matrix(from.Rx, from.Ry, from.Rz, rotSeq)
        : rotRadian2Matrix(from.Rx, from.Ry, from.Rz, rotSeq);
    const cv::Mat toRot = isDegree
        ? rotDegree2Matrix(to.Rx, to.Ry, to.Rz, rotSeq)
        : rotRadian2Matrix(to.Rx, to.Ry, to.Rz, rotSeq);

    // 旋转沿测地线插值：先用 Rodrigues 求出相对旋转的轴角（模长即相对转角，
    // 且必落在 [0, pi]），再按比例缩放该旋转向量，因此 t 在 [0,1] 内走最短路径，
    // 超出 [0,1] 时自然退化为外插。
    // 注意：相对转角恰为 pi 时轴向符号本身不唯一，插值路径由 cv::Rodrigues 的实现决定。
    const cv::Mat relRotMatrix = fromRot.t() * toRot;

    cv::Mat relRotVec;
    cv::Rodrigues(relRotMatrix, relRotVec);

    cv::Mat stepRotVec = t * relRotVec;
    cv::Mat stepRotMatrix;
    cv::Rodrigues(stepRotVec, stepRotMatrix);

    const cv::Mat rotMatrix = fromRot * stepRotMatrix;

    // 平移按线性插值
    cv::Mat hmMatrix = cv::Mat::eye(4, 4, CV_64F);
    rotMatrix.copyTo(hmMatrix(cv::Rect(0, 0, 3, 3)));
    hmMatrix.at<double>(0, 3) = (1.0 - t) * from.X + t * to.X;
    hmMatrix.at<double>(1, 3) = (1.0 - t) * from.Y + t * to.Y;
    hmMatrix.at<double>(2, 3) = (1.0 - t) * from.Z + t * to.Z;

    return hmMatrix2Pose(hmMatrix, outPose, rotSeq, angleType);
}