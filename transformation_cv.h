/*****************************************************************************
*  @file     transformation_cv.h                                             *
*  @brief    ��OpenCVʵ����ά�ռ���λ�������֮��ı任                         *
*  Details.  ���������ת�ǡ���ת������ξ���֮��ı任						 *
*			 �ڹ̶�������ϵ��������ת��˳��									 *
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
	/// �Ƕ�ת����
	/// </summary>
	/// <param name="degree">�Ƕȣ�-180 180��</param>
	/// <returns>���ȣ�-3.14 3.14��</returns>
	static double degree2Radian(const double& degree);

	/// <summary>
	/// ����ת�Ƕ�
	/// </summary>
	/// <param name="radian">���ȣ�-3.14 3.14��</param>
	/// <returns>�Ƕȣ�-180 180��</returns>
	static double radian2Degree(const double& radian);

	/// <summary>
	/// ��ת�ǣ�degree��-> ��ת���� Ĭ�ϰ���X-Y-Z�̶�������ϵ����Z-Y-Xŷ��������ϵ ���ߵȼۣ�ת��
	/// </summary>
	/// <param name="rx">��X����ת��</param>
	/// <param name="ry">��Y����ת��</param>
	/// <param name="rz">��Z����ת��</param>
	/// <param name="rotSeq">�̶�������ϵ�����µ���ת˳����ŷ��������ϵ����</param>
	/// <returns>��ת����3x3��</returns>
	static cv::Mat rotDegree2Matrix(double rx, double ry, double rz, const E_ROTATION_SEQUENCE& rotSeq = E_ROTATION_SEQUENCE::E_SEQ_XYZ);

	/// <summary>
	/// ��ת�ǣ�radian��-> ��ת���� Ĭ�ϰ���X-Y-Z�̶�������ϵ����Z-Y-Xŷ��������ϵ ���ߵȼۣ�ת��
	/// </summary>
	/// <param name="rx">��X����ת��</param>
	/// <param name="ry">��Y����ת��</param>
	/// <param name="rz">��Z����ת��</param>
	/// <param name="rotSeq">�̶�������ϵ�����µ���ת˳����ŷ��������ϵ����</param>
	/// <returns>��ת����3x3��</returns>
	static cv::Mat rotRadian2Matrix(double rx, double ry, double rz, const E_ROTATION_SEQUENCE& rotSeq = E_ROTATION_SEQUENCE::E_SEQ_XYZ);

	/// <summary>
	/// ��ת���� -> ��ת�ǣ�degree��Ĭ�ϰ���X-Y-Z�̶�������ϵ����Z-Y-Xŷ��������ϵ ���ߵȼۣ�ת��
	/// </summary>
	/// <param name="matrix">��ת����</param>
	/// <param name="rotSeq">�̶�������ϵ�����µ���ת˳����ŷ��������ϵ����</param>
	/// <returns>��ת�ǣ�degree��</returns>
	static cv::Vec3d rotMatrix2RotDegree(const cv::Mat& matrix, const E_ROTATION_SEQUENCE& rotSeq = E_ROTATION_SEQUENCE::E_SEQ_XYZ);

	/// <summary>
	/// ��ת���� -> ��ת�ǣ�Radian��Ĭ�ϰ���X-Y-Z�̶�������ϵ����Z-Y-Xŷ��������ϵ ���ߵȼۣ�ת��
	/// </summary>
	/// <param name="matrix">��ת����</param>
	/// <param name="rotSeq">�̶�������ϵ�����µ���ת˳����ŷ��������ϵ����</param>
	/// <returns>��ת�ǣ�Radian��</returns>
	static cv::Vec3d rotMatrix2RotRadian(const cv::Mat& matrix, const E_ROTATION_SEQUENCE& rotSeq = E_ROTATION_SEQUENCE::E_SEQ_XYZ);

	/// <summary>
	/// λ�� -> ��ξ���
	/// </summary>
	/// <param name="x">X</param>
	/// <param name="y">Y</param>
	/// <param name="z">Z</param>
	/// <param name="rx">Rx</param>
	/// <param name="ry">Ry</param>
	/// <param name="rz">Rz</param>
	/// <param name="rotSeq">��ת��˳�� Ĭ�ϰ���X-Y-Z�̶�������ϵ����Z-Y-Xŷ��������ϵ ���ߵȼۣ� </param>
	/// <param name="angleType">�Ƕȵ�λ Ĭ��Ϊdegree</param>
	/// <returns>��ξ���4x4��</returns>
	static cv::Mat pose2HmMatrix(double x, double y, double z, double rx, double ry, double rz,
		const E_ROTATION_SEQUENCE& rotSeq = E_ROTATION_SEQUENCE::E_SEQ_XYZ,
		const E_ANGLE_TYPE& angleType = E_ANGLE_TYPE::E_TYPE_DEGREE);

	/// <summary>
	/// λ��ת��ξ���
	/// </summary>
	/// <param name="pose">λ�ˣ�����X Y Z Rx Ry Rz��</param>
	/// <param name="rotSeq">��ת��˳�� Ĭ�ϰ���X-Y-Z�̶�������ϵ����Z-Y-Xŷ��������ϵ ���ߵȼۣ�</param>
	/// <param name="angleType">�Ƕȵ�λ Ĭ��Ϊdegree</param>
	/// <returns>��ξ���4x4��</returns>
	static cv::Mat pose2HmMatrix(const S_POSE& pose,
		const E_ROTATION_SEQUENCE& rotSeq = E_ROTATION_SEQUENCE::E_SEQ_XYZ,
		const E_ANGLE_TYPE& angleType = E_ANGLE_TYPE::E_TYPE_DEGREE);

	/// <summary>
	/// ��ξ���תΪλ��
	/// </summary>
	/// <param name="hmMatrix">��ξ���4x4��</param>
	/// <param name="rotSeq">��ת��˳�� Ĭ�ϰ���X-Y-Z�̶�������ϵ����Z-Y-Xŷ��������ϵ ���ߵȼۣ�</param>
	/// <param name="angleType">�Ƕȵ�λ Ĭ��Ϊdegree</param>
	/// <returns>λ�ˣ�����X Y Z Rx Ry Rz��</returns>
	static S_POSE hmMatrix2Pose(const cv::Mat& hmMatrix,
		const E_ROTATION_SEQUENCE& rotSeq = E_ROTATION_SEQUENCE::E_SEQ_XYZ,
		const E_ANGLE_TYPE& angleType = E_ANGLE_TYPE::E_TYPE_DEGREE);
};

#endif // !TRANSFORMATION_CV_H

