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

class CTransformation_EG
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
	/// <param name="x">X</param>
	/// <param name="y">Y</param>
	/// <param name="z">Z</param>
	/// <param name="rx">Rx</param>
	/// <param name="ry">Ry</param>
	/// <param name="rz">Rz</param>
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

	/// <summary>
	/// 通过Rodrigues将旋转向量转为矩阵
	/// </summary>
	/// <param name="k1">旋转向量1（弧度制）</param>
	/// <param name="k2">旋转向量2（弧度制）</param>
	/// <param name="k3">旋转向量3（弧度制）</param>
	/// <returns>旋转矩阵</returns>
	static Eigen::Matrix3d rotRadian2Matrix_Rodrigues(const double k1, const double k2, const double k3);

	/// <summary>
	/// 通过Rodrigues将旋转向量转为矩阵
	/// </summary>
	/// <param name="k1">旋转向量1（角度制）</param>
	/// <param name="k2">旋转向量2（角度制）</param>
	/// <param name="k3">旋转向量3（角度制）</param>
	/// <returns>旋转矩阵</returns>
	static Eigen::Matrix3d rotDegree2Matrix_Rodrigues(const double k1, const double k2, const double k3);

	/// <summary>
	/// 通过Rodrigues将旋转矩阵转为旋转向量
	/// </summary>
	/// <param name="rotMat">旋转矩阵</param>
	/// <returns>旋转向量（弧度制）；输入非法时静默返回零向量，需要区分请使用下面的 bool 重载</returns>
	static Eigen::Vector3d rotMatrix2RotRadian_Rodrigues(const Eigen::Matrix3d& rotMat);

	/// <summary>
	/// 通过Rodrigues将旋转矩阵转为旋转向量（可区分失败）
	/// </summary>
	/// <param name="rotMat">旋转矩阵（3x3，需满足正交且行列式为1）</param>
	/// <param name="rotVec">输出旋转向量（弧度制）；失败时置为零向量</param>
	/// <returns>成功返回true；矩阵非法返回false</returns>
	static bool rotMatrix2RotRadian_Rodrigues(const Eigen::Matrix3d& rotMat, Eigen::Vector3d& rotVec);

	/// <summary>
	/// 通过Rodrigues将旋转矩阵转为旋转向量
	/// </summary>
	/// <param name="rotMat">旋转矩阵</param>
	/// <returns>旋转向量（角度制）；输入非法时静默返回零向量，需要区分请使用下面的 bool 重载</returns>
	static Eigen::Vector3d rotMatrix2RotDegree_Rodrigues(const Eigen::Matrix3d& rotMat);

	/// <summary>
	/// 通过Rodrigues将旋转矩阵转为旋转向量（可区分失败）
	/// </summary>
	/// <param name="rotMat">旋转矩阵（3x3，需满足正交且行列式为1）</param>
	/// <param name="rotVec">输出旋转向量（角度制）；失败时置为零向量</param>
	/// <returns>成功返回true；矩阵非法返回false</returns>
	static bool rotMatrix2RotDegree_Rodrigues(const Eigen::Matrix3d& rotMat, Eigen::Vector3d& rotVec);

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
	static Eigen::Matrix4d pose2HmMatrix_Rodrigues(const double x, const double y, const double z, const double k1, const double k2, const double k3, const E_ANGLE_TYPE& angleType = E_ANGLE_TYPE::E_TYPE_DEGREE);

	/// <summary>
	/// 位姿转为齐次矩阵 旋转矩阵部分用Rodrigues实现
	/// </summary>
	/// <param name="pose">位姿</param>
	/// <param name="angleType">角度/弧度</param>
	/// <returns>齐次矩阵</returns>
	static Eigen::Matrix4d pose2HmMatrix_Rodrigues(const S_POSE& pose, const E_ANGLE_TYPE& angleType = E_ANGLE_TYPE::E_TYPE_DEGREE);
	/// <summary>
	/// 旋转矩阵 -> 旋转角（弧度）可区分失败
	/// </summary>
	/// <param name="matrix">旋转矩阵（3x3，需满足正交且行列式为1）</param>
	/// <param name="rotSeq">旋转角顺序</param>
	/// <param name="rotVec">输出旋转角（弧度制）；失败时置为零向量</param>
	/// <returns>成功返回true；矩阵非法返回false</returns>
	static bool rotMatrix2RotRadian(const Eigen::Matrix3d& matrix, const E_ROTATION_SEQUENCE& rotSeq, Eigen::Vector3d& rotVec);

	/// <summary>
	/// 旋转矩阵 -> 旋转角（角度）可区分失败
	/// </summary>
	/// <param name="matrix">旋转矩阵（3x3，需满足正交且行列式为1）</param>
	/// <param name="rotSeq">旋转角顺序</param>
	/// <param name="rotVec">输出旋转角（角度制）；失败时置为零向量</param>
	/// <returns>成功返回true；矩阵非法返回false</returns>
	static bool rotMatrix2RotDegree(const Eigen::Matrix3d& matrix, const E_ROTATION_SEQUENCE& rotSeq, Eigen::Vector3d& rotVec);

	/// <summary>
	/// 齐次矩阵转为位姿（可区分失败）
	/// </summary>
	/// <param name="hmMatrix">齐次矩阵（4x4，需满足旋转块正交且最后一行为[0 0 0 1]）</param>
	/// <param name="pose">输出位姿；失败时各分量置零</param>
	/// <param name="rotSeq">旋转角顺序 默认按照X-Y-Z固定角坐标系（或Z-Y-X欧拉角坐标系 二者等价）</param>
	/// <param name="angleType">角度单位 默认为degree</param>
	/// <returns>成功返回true；矩阵非法返回false</returns>
	static bool hmMatrix2Pose(const Eigen::Matrix4d& hmMatrix, S_POSE& pose,
		const E_ROTATION_SEQUENCE& rotSeq = E_ROTATION_SEQUENCE::E_SEQ_XYZ,
		const E_ANGLE_TYPE& angleType = E_ANGLE_TYPE::E_TYPE_DEGREE);

	/// <summary>
	/// 齐次矩阵求逆（刚体变换解析解：R^-1 = R^T，t^-1 = -R^T * t）
	/// </summary>
	/// <param name="hmMatrix">齐次矩阵（4x4）</param>
	/// <param name="invMatrix">输出逆矩阵（4x4）；失败时置为单位阵</param>
	/// <returns>成功返回true；矩阵非法返回false</returns>
	static bool invHmMatrix(const Eigen::Matrix4d& hmMatrix, Eigen::Matrix4d& invMatrix);

	/// <summary>
	/// 位姿求逆
	/// </summary>
	/// <param name="pose">输入位姿</param>
	/// <param name="outPose">输出逆位姿；失败时各分量置零</param>
	/// <param name="rotSeq">旋转角顺序</param>
	/// <param name="angleType">角度单位</param>
	/// <returns>成功返回true</returns>
	static bool invPose(const S_POSE& pose, S_POSE& outPose,
		const E_ROTATION_SEQUENCE& rotSeq = E_ROTATION_SEQUENCE::E_SEQ_XYZ,
		const E_ANGLE_TYPE& angleType = E_ANGLE_TYPE::E_TYPE_DEGREE);

	/// <summary>
	/// 齐次矩阵复合：outMatrix = a * b（先作用b，再作用a）
	/// </summary>
	/// <param name="a">左乘矩阵（4x4）</param>
	/// <param name="b">右乘矩阵（4x4）</param>
	/// <param name="outMatrix">输出复合矩阵（4x4）；失败时置为单位阵</param>
	/// <returns>成功返回true；任一矩阵非法返回false</returns>
	static bool composeHmMatrix(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b, Eigen::Matrix4d& outMatrix);

	/// <summary>
	/// 位姿复合：outPose = a * b（先作用b，再作用a）
	/// </summary>
	/// <param name="a">左乘位姿</param>
	/// <param name="b">右乘位姿</param>
	/// <param name="outPose">输出复合位姿；失败时各分量置零</param>
	/// <param name="rotSeq">旋转角顺序</param>
	/// <param name="angleType">角度单位</param>
	/// <returns>成功返回true</returns>
	static bool composePose(const S_POSE& a, const S_POSE& b, S_POSE& outPose,
		const E_ROTATION_SEQUENCE& rotSeq = E_ROTATION_SEQUENCE::E_SEQ_XYZ,
		const E_ANGLE_TYPE& angleType = E_ANGLE_TYPE::E_TYPE_DEGREE);

	/// <summary>
	/// 位姿插值：旋转沿测地线（最短路径）插值，平移按线性插值
	/// </summary>
	/// <param name="from">起始位姿（t=0）</param>
	/// <param name="to">目标位姿（t=1）</param>
	/// <param name="t">插值参数，允许超出[0,1]表示外插</param>
	/// <param name="outPose">输出位姿；失败时各分量置零</param>
	/// <param name="rotSeq">旋转角顺序</param>
	/// <param name="angleType">角度单位</param>
	/// <returns>成功返回true</returns>
	static bool interpolatePose(const S_POSE& from, const S_POSE& to, double t, S_POSE& outPose,
		const E_ROTATION_SEQUENCE& rotSeq = E_ROTATION_SEQUENCE::E_SEQ_XYZ,
		const E_ANGLE_TYPE& angleType = E_ANGLE_TYPE::E_TYPE_DEGREE);

private:
	/// <summary>
	/// 校验3x3旋转矩阵（正交且行列式为1）
	/// </summary>
	static bool isValidRotationMatrix(const Eigen::Matrix3d& rotMatrix);

	/// <summary>
	/// 校验4x4齐次矩阵（旋转块合法且最后一行为[0 0 0 1]）
	/// </summary>
	static bool isValidHmMatrix(const Eigen::Matrix4d& hmMatrix);
};

#endif // !TRANSFORMATION_EG_H
