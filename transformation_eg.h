/*****************************************************************************
*  @file     transformation_eg.h                                             *
*  @brief    用Eigen实现三维空间中位姿与矩阵之间的变换                          *
*  Details.  具体包括旋转角、旋转矩阵、齐次矩阵之间的变换						 *
*			 在固定角坐标系下描述旋转角顺序									 *
*                                                                            *
*  @author   Chan_XM                                                         *
*  @email    chan_xm14@163.com			                                     *
*  @version  V1.0														     *
*  @date     2024/11/16													     *
*  @license  NON															 *
*****************************************************************************/

#ifndef TRANSFORMATION_EG_H
#define TRANSFORMATION_EG_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "common.h"

class CTransformation
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
	static Eigen::Matrix3d rotDegree2Matrix(double rx, double ry, double rz, const E_ROTATION_SEQUENCE& rotSeq = E_ROTATION_SEQUENCE::E_SEQ_XYZ);

	/// <summary>
	/// 旋转角（radian）-> 旋转矩阵 默认按照X-Y-Z固定角坐标系（或Z-Y-X欧拉角坐标系 二者等价）转换
	/// </summary>
	/// <param name="rx">绕X轴旋转角</param>
	/// <param name="ry">绕Y轴旋转角</param>
	/// <param name="rz">绕Z轴旋转角</param>
	/// <param name="rotSeq">固定角坐标系描述下的旋转顺序（与欧拉角坐标系反向）</param>
	/// <returns>旋转矩阵（3x3）</returns>
	static Eigen::Matrix3d rotRadian2Matrix(double rx, double ry, double rz, const E_ROTATION_SEQUENCE& rotSeq = E_ROTATION_SEQUENCE::E_SEQ_XYZ);

	/// <summary>
	/// 旋转矩阵 -> 旋转角（degree）默认按照X-Y-Z固定角坐标系（或Z-Y-X欧拉角坐标系 二者等价）转换
	/// </summary>
	/// <param name="matrix">旋转矩阵</param>
	/// <param name="rotSeq">固定角坐标系描述下的旋转顺序（与欧拉角坐标系反向）</param>
	/// <returns>旋转角（degree）</returns>
	static Eigen::Vector3d rotMatrix2RotDegree(const Eigen::Matrix3d& matrix, const E_ROTATION_SEQUENCE& rotSeq = E_ROTATION_SEQUENCE::E_SEQ_XYZ);

	/// <summary>
	/// 旋转矩阵 -> 旋转角（Radian）默认按照X-Y-Z固定角坐标系（或Z-Y-X欧拉角坐标系 二者等价）转换
	/// </summary>
	/// <param name="matrix">旋转矩阵</param>
	/// <param name="rotSeq">固定角坐标系描述下的旋转顺序（与欧拉角坐标系反向）</param>
	/// <returns>旋转角（Radian）</returns>
	static Eigen::Vector3d rotMatrix2RotRadian(const Eigen::Matrix3d& matrix, const E_ROTATION_SEQUENCE& rotSeq = E_ROTATION_SEQUENCE::E_SEQ_XYZ);

	/// <summary>
	/// 位姿 -> 齐次矩阵
	/// </summary>
	/// <param name="x"></param>
	/// <param name="y"></param>
	/// <param name="z"></param>
	/// <param name="rx"></param>
	/// <param name="ry"></param>
	/// <param name="rz"></param>
	/// <param name="rotSeq">旋转角顺序 默认按照X-Y-Z固定角坐标系（或Z-Y-X欧拉角坐标系 二者等价） </param>
	/// <param name="angleType">角度单位 默认为degree</param>
	/// <returns>齐次矩阵（4x4）</returns>
	static Eigen::Matrix4d pose2HmMatrix(double x, double y, double z, double rx, double ry, double rz,
		const E_ROTATION_SEQUENCE& rotSeq = E_ROTATION_SEQUENCE::E_SEQ_XYZ,
		const E_ANGLE_TYPE& angleType = E_ANGLE_TYPE::E_TYPE_DEGREE);
	
	/// <summary>
	/// 位姿转齐次矩阵
	/// </summary>
	/// <param name="pose">位姿（包含X Y Z Rx Ry Rz）</param>
	/// <param name="rotSeq">旋转角顺序 默认按照X-Y-Z固定角坐标系（或Z-Y-X欧拉角坐标系 二者等价）</param>
	/// <param name="angleType">角度单位 默认为degree</param>
	/// <returns>齐次矩阵（4x4）</returns>
	static Eigen::Matrix4d pose2HmMatrix(const S_POSE& pose,
		const E_ROTATION_SEQUENCE& rotSeq = E_ROTATION_SEQUENCE::E_SEQ_XYZ,
		const E_ANGLE_TYPE& angleType = E_ANGLE_TYPE::E_TYPE_DEGREE);

	/// <summary>
	/// 齐次矩阵转为位姿
	/// </summary>
	/// <param name="hmMatrix">齐次矩阵（4x4）</param>
	/// <param name="rotSeq">旋转角顺序 默认按照X-Y-Z固定角坐标系（或Z-Y-X欧拉角坐标系 二者等价）</param>
	/// <param name="angleType">角度单位 默认为degree</param>
	/// <returns>位姿（包含X Y Z Rx Ry Rz）</returns>
	static S_POSE hmMatrix2Pose(const Eigen::Matrix4d& hmMatrix, 
		const E_ROTATION_SEQUENCE& rotSeq = E_ROTATION_SEQUENCE::E_SEQ_XYZ,
		const E_ANGLE_TYPE& angleType = E_ANGLE_TYPE::E_TYPE_DEGREE);

};

#endif // !TRANSFORMATION_EG_H
