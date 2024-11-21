#include "transformation_cv.h"

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

cv::Vec3d CTransformation_CV::rotMatrix2RotRadian(const cv::Mat& matrix, const E_ROTATION_SEQUENCE& rotSeq)
{
    if (matrix.cols != 3 || matrix.rows != 3)
    {
        std::cerr << "rotMatrix2RotRadian() matrix £¡= 3x3" << std::endl;
        return cv::Vec3d();
    }

    if (matrix.channels() != 1)
    {
        std::cerr << "rotMatrix2RotDegree() matrix channels != 1" << std::endl;
        return cv::Vec3d();
    }

    double R11, R12, R13, R21, R22, R23, R31, R32, R33;

    switch (CV_MAT_DEPTH(matrix.type()))
    {
    case CV_32F:
    {
        R11 = matrix.at<float>(0, 0);		R12 = matrix.at<float>(0, 1);		R13 = matrix.at<float>(0, 2);
        R21 = matrix.at<float>(1, 0);		R22 = matrix.at<float>(1, 1);		R23 = matrix.at<float>(1, 2);
        R31 = matrix.at<float>(2, 0);		R32 = matrix.at<float>(2, 1);		R33 = matrix.at<float>(2, 2);
        break;
    }
    case CV_64F:
    {
        R11 = matrix.at<double>(0, 0);		R12 = matrix.at<double>(0, 1);		R13 = matrix.at<double>(0, 2);
        R21 = matrix.at<double>(1, 0);		R22 = matrix.at<double>(1, 1);		R23 = matrix.at<double>(1, 2);
        R31 = matrix.at<double>(2, 0);		R32 = matrix.at<double>(2, 1);		R33 = matrix.at<double>(2, 2);
        break;
    }

    default:
    {
        std::cerr << "rotMatrix2RotDegree() matrix type != float/double" << std::endl;
        return cv::Vec3d();
    }
    }

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

        cv::Vec3d rotRadian = { gama, beta,alfa };
        return rotRadian;
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

        cv::Vec3d rotRadian = { alfa, beta, gama };
        return rotRadian;
    }
    else
    {
        return cv::Vec3d();
    }
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
    double x = 0.0, y = 0.0, z = 0.0, rx = 0.0, ry = 0.0, rz = 0.0;

    if (hmMatrix.cols != 4 || hmMatrix.rows != 4)
    {
        std::cerr << "hmMatrix2Pose() hmMatrix != 4x4" << std::endl;
        return S_POSE(x, y, z, rx, ry, rz);
    }

    if (hmMatrix.channels() != 1)
    {
        std::cerr << "hmMatrix2Pose() hmMatrix channels != 1" << std::endl;
        return S_POSE(x, y, z, rx, ry, rz);
    }

    cv::Mat rotMatrix = hmMatrix(cv::Range(0, 3), cv::Range(0, 3));

    std::cout << "rotMatrix = " << rotMatrix << std::endl << std::endl;

    cv::Vec3d angles;

    if (angleType == E_ANGLE_TYPE::E_TYPE_DEGREE)
    {
        angles = rotMatrix2RotDegree(rotMatrix, rotSeq);
    }
    else
    {
        angles = rotMatrix2RotRadian(rotMatrix, rotSeq);
    }

    rx = angles[0];     ry = angles[1];     rz = angles[2];

    switch (CV_MAT_DEPTH(hmMatrix.type()))
    {
    case CV_32F:
    {
        x = hmMatrix.at<float>(0, 3);
        y = hmMatrix.at<float>(1, 3);
        z = hmMatrix.at<float>(2, 3);
        break;
    }
    case CV_64F:
    {
        x = hmMatrix.at<double>(0, 3);
        y = hmMatrix.at<double>(1, 3);
        z = hmMatrix.at<double>(2, 3);
        break;
    }
    default:
    {
        std::cerr << "hmMatrix2Pose() matrix type != float/double" << std::endl;
        return S_POSE(x, y, z, rx, ry, rz);
    }
    }

    return S_POSE(x, y, z, rx, ry, rz);
}
