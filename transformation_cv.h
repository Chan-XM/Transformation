/*****************************************************************************
*  @file     transformation_cv.h                                             *
*  @brief    用OpenCV实现三维空间中位姿与矩阵之间的变换                         *
*  Details.  具体包括旋转角、旋转矩阵、齐次矩阵之间的变换						 *
*			 在固定角坐标系下描述旋转角顺序									 *
*                                                                            *
*  @author   Chan_XM                                                         *
*  @email    chan_xm14@163.com			                                     *
*  @version  V1.0														     *
*  @date     2024/11/20													     *
*  @license  NON															 *
*****************************************************************************/

#ifndef TRANSFORMATION_CV_H
#define TRANSFORMATION_CV_H

#include "common.h"

#include <opencv2/opencv.hpp>

class CTransformation_CV
{
public:
	/// <summary>
	/// 角度转弧度
	/// </summary>
	/// <param name="degree">角度（-180 180）</param>
	/// <returns>弧度（-3.14 3.14）</returns>
	static double degree2Radian(const double& degree);

	/// <summary>
	/// 弧度转角度
	/// </summary>
	/// <param name="radian">弧度（-3.14 3.14）</param>
	/// <returns>角度（-180 180）</returns>
	static double radian2Degree(const double& radian);

	/// <summary>
	/// 旋转角（degree）-> 旋转矩阵 默认按照X-Y-Z固定角坐标系（或Z-Y-X欧拉角坐标系 二者等价）转换
	/// </summary>
	/// <param name="rx">绕X轴旋转角</param>
	/// <param name="ry">绕Y轴旋转角</param>
	/// <param name="rz">绕Z轴旋转角</param>
	/// <param name="rotSeq">固定角坐标系描述下的旋转顺序（与欧拉角坐标系反向）</param>
	/// <returns>旋转矩阵（3x3）</returns>
	static cv::Mat rotDegree2Matrix(double rx, double ry, double rz, const E_ROTATION_SEQUENCE& rotSeq = E_ROTATION_SEQUENCE::E_SEQ_XYZ);

	/// <summary>
	/// 旋转角（radian）-> 旋转矩阵 默认按照X-Y-Z固定角坐标系（或Z-Y-X欧拉角坐标系 二者等价）转换
	/// </summary>
	/// <param name="rx">绕X轴旋转角</param>
	/// <param name="ry">绕Y轴旋转角</param>
	/// <param name="rz">绕Z轴旋转角</param>
	/// <param name="rotSeq">固定角坐标系描述下的旋转顺序（与欧拉角坐标系反向）</param>
	/// <returns>旋转矩阵（3x3）</returns>
	static cv::Mat rotRadian2Matrix(double rx, double ry, double rz, const E_ROTATION_SEQUENCE& rotSeq = E_ROTATION_SEQUENCE::E_SEQ_XYZ);

	/// <summary>
	/// 旋转矩阵 -> 旋转角（degree）默认按照X-Y-Z固定角坐标系（或Z-Y-X欧拉角坐标系 二者等价）转换
	/// </summary>
	/// <param name="matrix">旋转矩阵</param>
	/// <param name="rotSeq">固定角坐标系描述下的旋转顺序（与欧拉角坐标系反向）</param>
	/// <returns>旋转角（degree）</returns>
	static cv::Vec3d rotMatrix2RotDegree(const cv::Mat& matrix, const E_ROTATION_SEQUENCE& rotSeq = E_ROTATION_SEQUENCE::E_SEQ_XYZ);

	/// <summary>
	/// 旋转矩阵 -> 旋转角（Radian）默认按照X-Y-Z固定角坐标系（或Z-Y-X欧拉角坐标系 二者等价）转换
	/// </summary>
	/// <param name="matrix">旋转矩阵</param>
	/// <param name="rotSeq">固定角坐标系描述下的旋转顺序（与欧拉角坐标系反向）</param>
	/// <returns>旋转角（Radian）</returns>
	static cv::Vec3d rotMatrix2RotRadian(const cv::Mat& matrix, const E_ROTATION_SEQUENCE& rotSeq = E_ROTATION_SEQUENCE::E_SEQ_XYZ);

	/// <summary>
	/// 位姿 -> 齐次矩阵
	/// </summary>
	/// <param name="x">X</param>
	/// <param name="y">Y</param>
	/// <param name="z">Z</param>
	/// <param name="rx">Rx</param>
	/// <param name="ry">Ry</param>
	/// <param name="rz">Rz</param>
	/// <param name="rotSeq">旋转角顺序 默认按照X-Y-Z固定角坐标系（或Z-Y-X欧拉角坐标系 二者等价） </param>
	/// <param name="angleType">角度单位 默认为degree</param>
	/// <returns>齐次矩阵（4x4）</returns>
	static cv::Mat pose2HmMatrix(double x, double y, double z, double rx, double ry, double rz,
		const E_ROTATION_SEQUENCE& rotSeq = E_ROTATION_SEQUENCE::E_SEQ_XYZ,
		const E_ANGLE_TYPE& angleType = E_ANGLE_TYPE::E_TYPE_DEGREE);

	/// <summary>
	/// 位姿转齐次矩阵
	/// </summary>
	/// <param name="pose">位姿（包含X Y Z Rx Ry Rz）</param>
	/// <param name="rotSeq">旋转角顺序 默认按照X-Y-Z固定角坐标系（或Z-Y-X欧拉角坐标系 二者等价）</param>
	/// <param name="angleType">角度单位 默认为degree</param>
	/// <returns>齐次矩阵（4x4）</returns>
	static cv::Mat pose2HmMatrix(const S_POSE& pose,
		const E_ROTATION_SEQUENCE& rotSeq = E_ROTATION_SEQUENCE::E_SEQ_XYZ,
		const E_ANGLE_TYPE& angleType = E_ANGLE_TYPE::E_TYPE_DEGREE);

	/// <summary>
	/// 齐次矩阵转为位姿
	/// </summary>
	/// <param name="hmMatrix">齐次矩阵（4x4）</param>
	/// <param name="rotSeq">旋转角顺序 默认按照X-Y-Z固定角坐标系（或Z-Y-X欧拉角坐标系 二者等价）</param>
	/// <param name="angleType">角度单位 默认为degree</param>
	/// <returns>位姿（包含X Y Z Rx Ry Rz）</returns>
	static S_POSE hmMatrix2Pose(const cv::Mat& hmMatrix,
		const E_ROTATION_SEQUENCE& rotSeq = E_ROTATION_SEQUENCE::E_SEQ_XYZ,
		const E_ANGLE_TYPE& angleType = E_ANGLE_TYPE::E_TYPE_DEGREE);

	/// <summary>
	/// 通过Rodrigues将旋转向量转为矩阵
	/// </summary>
	/// <param name="k1">旋转向量1（弧度制）</param>
	/// <param name="k2">旋转向量2（弧度制）</param>
	/// <param name="k3">旋转向量3（弧度制）</param>
	/// <returns>旋转矩阵</returns>
	static cv::Mat rotRadian2Matrix_Rodrigues(const double k1, const double k2, const double k3);

	/// <summary>
	/// 通过Rodrigues将旋转向量转为矩阵
	/// </summary>
	/// <param name="k1">旋转向量1（角度制）</param>
	/// <param name="k2">旋转向量2（角度制）</param>
	/// <param name="k3">旋转向量3（角度制）</param>
	/// <returns>旋转矩阵</returns>
	static cv::Mat rotDegree2Matrix_Rodrigues(const double k1, const double k2, const double k3);

	/// <summary>
	/// 通过Rodrigues将旋转矩阵转为旋转向量
	/// </summary>
	/// <param name="rotMat">旋转矩阵</param>
	/// <returns>旋转向量（弧度制）；输入非法时静默返回零向量，需要区分请使用下面的 bool 重载</returns>
	static cv::Vec3d rotMatrix2RotRadian_Rodrigues(const cv::Mat& rotMat);

	/// <summary>
	/// 通过Rodrigues将旋转矩阵转为旋转向量（可区分失败）
	/// </summary>
	/// <param name="rotMat">旋转矩阵（3x3，需满足正交且行列式为1）</param>
	/// <param name="rotVec">输出旋转向量（弧度制）；失败时置为零向量</param>
	/// <returns>成功返回true；矩阵非法返回false</returns>
	static bool rotMatrix2RotRadian_Rodrigues(const cv::Mat& rotMat, cv::Vec3d& rotVec);

	/// <summary>
	/// 通过Rodrigues将旋转矩阵转为旋转向量
	/// </summary>
	/// <param name="rotMat">旋转矩阵</param>
	/// <returns>旋转向量（角度制）；输入非法时静默返回零向量，需要区分请使用下面的 bool 重载</returns>
	static cv::Vec3d rotMatrix2RotDegree_Rodrigues(const cv::Mat& rotMat);

	/// <summary>
	/// 通过Rodrigues将旋转矩阵转为旋转向量（可区分失败）
	/// </summary>
	/// <param name="rotMat">旋转矩阵（3x3，需满足正交且行列式为1）</param>
	/// <param name="rotVec">输出旋转向量（角度制）；失败时置为零向量</param>
	/// <returns>成功返回true；矩阵非法返回false</returns>
	static bool rotMatrix2RotDegree_Rodrigues(const cv::Mat& rotMat, cv::Vec3d& rotVec);

	/// <summary>
	/// 位姿转为齐次矩阵 旋转矩阵部分用Rodrigues实现
	/// </summary>
	/// <param name="x">平移向量1</param>
	/// <param name="y">平移向量2</param>
	/// <param name="z">平移向量3</param>
	/// <param name="k1">旋转向量1</param>
	/// <param name="k2">旋转向量2</param>
	/// <param name="k3">旋转向量3</param>
	/// <param name="angleType">角度/弧度</param>
	/// <returns>齐次矩阵</returns>
	static cv::Mat pose2HmMatrix_Rodrigues(const double x, const double y, const double z, const double k1, const double k2, const double k3, const E_ANGLE_TYPE& angleType = E_ANGLE_TYPE::E_TYPE_DEGREE);

	/// <summary>
	/// 位姿转为齐次矩阵 旋转矩阵部分用Rodrigues实现
	/// </summary>
	/// <param name="pose">位姿</param>
	/// <param name="angleType">角度/弧度</param>
	/// <returns>齐次矩阵</returns>
	static cv::Mat pose2HmMatrix_Rodrigues(const S_POSE& pose, const E_ANGLE_TYPE& angleType = E_ANGLE_TYPE::E_TYPE_DEGREE);
	/// <summary>
	/// 旋转矩阵 -> 旋转角（弧度）可区分失败
	/// </summary>
	/// <param name="matrix">旋转矩阵（3x3，需满足正交且行列式为1）</param>
	/// <param name="rotSeq">旋转角顺序</param>
	/// <param name="rotVec">输出旋转角（弧度制）；失败时置为零向量</param>
	/// <returns>成功返回true；矩阵非法返回false</returns>
	static bool rotMatrix2RotRadian(const cv::Mat& matrix, const E_ROTATION_SEQUENCE& rotSeq, cv::Vec3d& rotVec);

	/// <summary>
	/// 旋转矩阵 -> 旋转角（角度）可区分失败
	/// </summary>
	/// <param name="matrix">旋转矩阵（3x3，需满足正交且行列式为1）</param>
	/// <param name="rotSeq">旋转角顺序</param>
	/// <param name="rotVec">输出旋转角（角度制）；失败时置为零向量</param>
	/// <returns>成功返回true；矩阵非法返回false</returns>
	static bool rotMatrix2RotDegree(const cv::Mat& matrix, const E_ROTATION_SEQUENCE& rotSeq, cv::Vec3d& rotVec);

	/// <summary>
	/// 齐次矩阵转为位姿（可区分失败）
	/// </summary>
	/// <param name="hmMatrix">齐次矩阵（4x4，需满足旋转块正交且最后一行为[0 0 0 1]）</param>
	/// <param name="pose">输出位姿；失败时各分量置零</param>
	/// <param name="rotSeq">旋转角顺序 默认按照X-Y-Z固定角坐标系（或Z-Y-X欧拉角坐标系 二者等价）</param>
	/// <param name="angleType">角度单位 默认为degree</param>
	/// <returns>成功返回true；矩阵非法返回false</returns>
	static bool hmMatrix2Pose(const cv::Mat& hmMatrix, S_POSE& pose,
		const E_ROTATION_SEQUENCE& rotSeq = E_ROTATION_SEQUENCE::E_SEQ_XYZ,
		const E_ANGLE_TYPE& angleType = E_ANGLE_TYPE::E_TYPE_DEGREE);

private:
	/// <summary>
	/// 校验3x3旋转矩阵的形状/通道/深度，并统一转为CV_64F
	/// </summary>
	static bool toRotMatrix64(const cv::Mat& rotMatrix, cv::Mat& rotMatrix64);

	/// <summary>
	/// 校验4x4齐次矩阵的形状/通道/深度，并统一转为CV_64F
	/// </summary>
	static bool toHmMatrix64(const cv::Mat& hmMatrix, cv::Mat& hmMatrix64);

	/// <summary>
	/// 校验CV_64F的3x3旋转矩阵（正交且行列式为1）
	/// </summary>
	static bool isValidRotationMatrix64(const cv::Mat& rotMatrix64);

	/// <summary>
	/// 校验CV_64F的4x4齐次矩阵（旋转块合法且最后一行为[0 0 0 1]）
	/// </summary>
	static bool isValidHmMatrix64(const cv::Mat& hmMatrix64);

	/// <summary>
	/// 近pi时固定旋转向量的轴向符号（与Eigen侧约定一致：绝对值最大的分量为正）
	/// </summary>
	static void canonicalizeNearPiAxis(cv::Vec3d& rotVec);
};


#endif // !TRANSFORMATION_CV_H

