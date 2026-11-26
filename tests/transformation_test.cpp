/*****************************************************************************
*  @file     transformation_test.cpp                                          *
*  @brief    位姿与矩阵变换的回归测试                                          *
*  Details.  覆盖旋转角、旋转矩阵、齐次矩阵、Rodrigues 旋转向量之间的往返，
*            并针对已修复的缺陷保留回归用例：
*            1) 非正交矩阵导致 acos 越界产生 NaN
*            2) OpenCV 侧 Rodrigues 逆变换传入 CV_32F / 4x4 时崩溃
*            3) 极小旋转向量被误判成零旋转（S_POSE 版角度单位漏传同属此类）
*            全部通过返回 0，否则返回失败用例个数                                *
*                                                                            *
*  @author   Chan_XM                                                         *
*  @date     2026/11/25                                                      *
*  @license  NON                                                             *
*****************************************************************************/

#include <iostream>
#include <iomanip>
#include <random>
#include <cmath>
#include <algorithm>

#include "transformation_eg.h"
#include "transformation_cv.h"

namespace
{
	int g_checked = 0;
	int g_failed = 0;

	void check(bool condition, const char* expr, int line)
	{
		++g_checked;
		if (!condition)
		{
			++g_failed;
			std::cout << "  [FAIL] 第 " << line << " 行: " << expr << std::endl;
		}
	}

	void section(const char* title)
	{
		std::cout << std::endl << "---- " << title << " ----" << std::endl;
	}

	void info(const char* name, double value)
	{
		std::cout << "  " << name << " = " << value << std::endl;
	}

	std::mt19937 g_rng(20241125);

	double uniform(double lo, double hi)
	{
		std::uniform_real_distribution<double> dist(lo, hi);
		return dist(g_rng);
	}

	Eigen::Vector3d randomUnitVector()
	{
		Eigen::Vector3d v(uniform(-1.0, 1.0), uniform(-1.0, 1.0), uniform(-1.0, 1.0));
		if (v.norm() < 1e-6)
		{
			v = Eigen::Vector3d::UnitX();
		}
		return v.normalized();
	}
}

#define CHECK(cond) check((cond), #cond, __LINE__)

// ===========================================================================
// 容差常量
// ===========================================================================
static void test_tolerance_constants()
{
	section("容差常量");

	// 校验容差必须比分支阈值宽松，否则真实数据会被判为非法
	CHECK(TRANSFORM_TOL::CHECK_ORTHOGONAL >= TRANSFORM_TOL::BRANCH_ZERO_ANGLE);

	// 零角度阈值必须足够小，不能把真实存在的小旋转抹掉（回归：曾被放宽到 1e-6）
	CHECK(TRANSFORM_TOL::BRANCH_ZERO_ANGLE < 1e-9);
}

// ===========================================================================
// Eigen 侧
// ===========================================================================
static void test_eg_angle_unit()
{
	section("EG: 角度/弧度互换");

	CHECK(std::abs(CTransformation_EG::degree2Radian(180.0) - M_PI) < 1e-12);
	CHECK(std::abs(CTransformation_EG::radian2Degree(M_PI) - 180.0) < 1e-12);
	CHECK(std::abs(CTransformation_EG::radian2Degree(CTransformation_EG::degree2Radian(-37.5)) + 37.5) < 1e-12);
}

static void test_eg_euler_roundtrip(E_ROTATION_SEQUENCE rotSeq, const char* title)
{
	section(title);

	double maxErr = 0.0;
	int bad = 0;
	for (int i = 0; i < 2000; ++i)
	{
		double rx = uniform(-180.0, 180.0);
		double ry = uniform(-80.0, 80.0);		// 避开万向锁
		double rz = uniform(-180.0, 180.0);

		Eigen::Matrix3d rotMat = CTransformation_EG::rotDegree2Matrix(rx, ry, rz, rotSeq);
		Eigen::Vector3d angles = CTransformation_EG::rotMatrix2RotDegree(rotMat, rotSeq);

		double err = std::max(std::max(std::abs(angles[0] - rx), std::abs(angles[1] - ry)), std::abs(angles[2] - rz));
		maxErr = std::max(maxErr, err);
		if (err > 1e-8)
		{
			++bad;
		}
	}

	info("最大角度误差(度)", maxErr);
	CHECK(bad == 0);
}

static void test_eg_gimbal_lock()
{
	section("EG: 万向锁 ry = ±90");

	for (int s = 0; s < 2; ++s)
	{
		E_ROTATION_SEQUENCE rotSeq = (s == 0) ? E_ROTATION_SEQUENCE::E_SEQ_XYZ : E_ROTATION_SEQUENCE::E_SEQ_ZYX;

		for (int t = 0; t < 2; ++t)
		{
			double ry = (t == 0) ? 90.0 : -90.0;

			Eigen::Matrix3d rotMat = CTransformation_EG::rotDegree2Matrix(30.0, ry, 45.0, rotSeq);
			Eigen::Vector3d angles = CTransformation_EG::rotMatrix2RotDegree(rotMat, rotSeq);
			Eigen::Matrix3d rebuild = CTransformation_EG::rotDegree2Matrix(angles[0], angles[1], angles[2], rotSeq);

			CHECK((rebuild - rotMat).norm() < 1e-12);
			CHECK(std::abs(angles[1] - ry) < 1e-9);
		}
	}
}

static void test_eg_pose_roundtrip()
{
	section("EG: 位姿 -> 齐次矩阵 -> 位姿");

	for (int s = 0; s < 2; ++s)
	{
		E_ROTATION_SEQUENCE rotSeq = (s == 0) ? E_ROTATION_SEQUENCE::E_SEQ_XYZ : E_ROTATION_SEQUENCE::E_SEQ_ZYX;

		double maxErr = 0.0;
		for (int i = 0; i < 1000; ++i)
		{
			S_POSE in(uniform(-1000.0, 1000.0), uniform(-1000.0, 1000.0), uniform(-1000.0, 1000.0),
				uniform(-180.0, 180.0), uniform(-80.0, 80.0), uniform(-180.0, 180.0));

			Eigen::Matrix4d hmMatrix = CTransformation_EG::pose2HmMatrix(in, rotSeq);
			S_POSE out = CTransformation_EG::hmMatrix2Pose(hmMatrix, rotSeq);

			double err = std::max(std::max(std::abs(out.X - in.X), std::abs(out.Y - in.Y)), std::abs(out.Z - in.Z));
			err = std::max(err, std::max(std::max(std::abs(out.Rx - in.Rx), std::abs(out.Ry - in.Ry)), std::abs(out.Rz - in.Rz)));
			maxErr = std::max(maxErr, err);
		}

		info("最大位姿误差", maxErr);
		CHECK(maxErr < 1e-8);
	}
}

static void test_eg_rodrigues_roundtrip()
{
	section("EG: Rodrigues 往返");

	double maxErr = 0.0;
	int rejected = 0;
	for (int i = 0; i < 2000; ++i)
	{
		double theta = uniform(-(M_PI - 0.1), (M_PI - 0.1));		// 避开 ±pi 的表示二义性
		Eigen::Vector3d rotVec = theta * randomUnitVector();

		Eigen::Matrix3d rotMat = CTransformation_EG::rotRadian2Matrix_Rodrigues(rotVec.x(), rotVec.y(), rotVec.z());

		Eigen::Vector3d back;
		if (!CTransformation_EG::rotMatrix2RotRadian_Rodrigues(rotMat, back))
		{
			++rejected;
			continue;
		}
		maxErr = std::max(maxErr, (back - rotVec).norm());
	}

	info("最大旋转向量误差(rad)", maxErr);
	CHECK(rejected == 0);
	CHECK(maxErr < 1e-9);
}

static void test_eg_rodrigues_near_pi()
{
	section("EG: Rodrigues 近 pi");

	// 普通分支：离 pi 足够远，轴向与角度都应准确
	double maxErr = 0.0;
	for (int i = 0; i < 1000; ++i)
	{
		double theta = M_PI - uniform(1e-4, 1e-3);
		Eigen::Vector3d rotVec = theta * randomUnitVector();
		Eigen::Matrix3d rotMat = CTransformation_EG::rotRadian2Matrix_Rodrigues(rotVec.x(), rotVec.y(), rotVec.z());

		Eigen::Vector3d back;
		CHECK(CTransformation_EG::rotMatrix2RotRadian_Rodrigues(rotMat, back));
		CHECK(!back.hasNaN());

		Eigen::Matrix3d rebuild = CTransformation_EG::rotRadian2Matrix_Rodrigues(back.x(), back.y(), back.z());
		maxErr = std::max(maxErr, (rebuild - rotMat).norm());
	}
	info("矩阵重建误差(普通分支)", maxErr);
	CHECK(maxErr < 1e-9);

	// 特殊分支：与 pi 的差值小于 BRANCH_NEAR_PI
	// 注意：R + I = 2*k*k^T 对 k 与 -k 不可区分，因此轴向符号二义，此处只要求矩阵可重建
	double maxErrPi = 0.0;
	for (int i = 0; i < 1000; ++i)
	{
		double theta = M_PI - uniform(0.0, 1e-7);
		Eigen::Vector3d rotVec = theta * randomUnitVector();
		Eigen::Matrix3d rotMat = CTransformation_EG::rotRadian2Matrix_Rodrigues(rotVec.x(), rotVec.y(), rotVec.z());

		Eigen::Vector3d back;
		CHECK(CTransformation_EG::rotMatrix2RotRadian_Rodrigues(rotMat, back));
		CHECK(!back.hasNaN());

		Eigen::Matrix3d rebuild = CTransformation_EG::rotRadian2Matrix_Rodrigues(back.x(), back.y(), back.z());
		maxErrPi = std::max(maxErrPi, (rebuild - rotMat).norm());
	}
	info("矩阵重建误差(pi 分支)", maxErrPi);
	CHECK(maxErrPi < 1e-5);
}

static void test_eg_small_angle()
{
	section("EG: 极小旋转不应被抹成零（回归）");

	Eigen::Vector3d axis = Eigen::Vector3d(0.6, 0.0, 0.8);
	double theta = 9e-7;						// 小于曾被错误放宽到的 1e-6

	Eigen::Matrix3d rotMat = CTransformation_EG::rotRadian2Matrix_Rodrigues(theta * axis.x(), theta * axis.y(), theta * axis.z());

	// 正向：矩阵必须真的偏离单位阵，而不是被直接判成单位阵
	CHECK((rotMat - Eigen::Matrix3d::Identity()).norm() > 0.5 * theta);

	// 反向：模长应能还原（acos 在小角度处精度有限，误差约 1e-10 量级）
	Eigen::Vector3d back;
	CHECK(CTransformation_EG::rotMatrix2RotRadian_Rodrigues(rotMat, back));
	CHECK(std::abs(back.norm() - theta) < 1e-8);
	info("还原的角度(rad)", back.norm());
}

static void test_eg_nan_vanish()
{
	section("EG: 非正交矩阵不应产生 NaN（回归）");

	// 该矩阵可以通过正交性/行列式校验，但 (trace - 1) / 2 会略微大于 1
	Eigen::Matrix3d stretched = (1.0 + 3e-7) * Eigen::Matrix3d::Identity();

	Eigen::Vector3d rotVec;
	CHECK(CTransformation_EG::rotMatrix2RotRadian_Rodrigues(stretched, rotVec));
	CHECK(!rotVec.hasNaN());

	// 兼容接口同样不应返回 NaN
	CHECK(!CTransformation_EG::rotMatrix2RotRadian_Rodrigues(stretched).hasNaN());
	CHECK(!CTransformation_EG::rotMatrix2RotDegree_Rodrigues(stretched).hasNaN());

	// 明显非正交的矩阵应被判为非法
	Eigen::Vector3d bad;
	CHECK(!CTransformation_EG::rotMatrix2RotRadian_Rodrigues(1.00001 * Eigen::Matrix3d::Identity(), bad));
	CHECK(!bad.hasNaN());
}

static void test_eg_invalid_input()
{
	section("EG: 非法输入可被区分（回归）");

	// 随机矩阵不是旋转矩阵
	Eigen::Vector3d rotVec(7.0, 8.0, 9.0);
	CHECK(!CTransformation_EG::rotMatrix2RotRadian_Rodrigues(Eigen::Matrix3d::Random(), rotVec));
	CHECK(rotVec.norm() == 0.0);				// 失败时输出被清零

	// 单位阵是合法输入，结果为零旋转
	Eigen::Vector3d zero(7.0, 8.0, 9.0);
	CHECK(CTransformation_EG::rotMatrix2RotRadian_Rodrigues(Eigen::Matrix3d::Identity(), zero));
	CHECK(zero.norm() == 0.0);

	// 行列式为 -1 的镜像矩阵应被判为非法
	Eigen::Matrix3d mirrored = Eigen::Matrix3d::Identity();
	mirrored(0, 0) = -1.0;
	CHECK(!CTransformation_EG::rotMatrix2RotRadian_Rodrigues(mirrored, rotVec));

	// 角度制接口同样可区分
	CHECK(!CTransformation_EG::rotMatrix2RotDegree_Rodrigues(Eigen::Matrix3d::Random(), rotVec));
}

static void test_eg_rodrigues_pose_angle_type()
{
	section("EG: S_POSE 版 Rodrigues 的角度单位透传（回归）");

	S_POSE poseRadian(1.0, 2.0, 3.0, 0.0, 0.0, M_PI / 2);
	S_POSE poseDegree(1.0, 2.0, 3.0, 0.0, 0.0, 90.0);

	Eigen::Matrix4d byRadian = CTransformation_EG::pose2HmMatrix_Rodrigues(poseRadian, E_ANGLE_TYPE::E_TYPE_RADIAN);
	Eigen::Matrix4d byDegree = CTransformation_EG::pose2HmMatrix_Rodrigues(poseDegree, E_ANGLE_TYPE::E_TYPE_DEGREE);

	CHECK((byRadian - byDegree).norm() < 1e-12);

	// 绕 Z 轴 90 度：R = [0 -1 0; 1 0 0; 0 0 1]
	CHECK(std::abs(byRadian(0, 1) + 1.0) < 1e-12);
	CHECK(std::abs(byRadian(1, 0) - 1.0) < 1e-12);
}

// ===========================================================================
// OpenCV 侧
// ===========================================================================
static void test_cv_euler_roundtrip()
{
	section("CV: 欧拉角往返");

	double maxErr = 0.0;
	for (int i = 0; i < 2000; ++i)
	{
		double rx = uniform(-180.0, 180.0);
		double ry = uniform(-80.0, 80.0);
		double rz = uniform(-180.0, 180.0);

		for (int s = 0; s < 2; ++s)
		{
			E_ROTATION_SEQUENCE rotSeq = (s == 0) ? E_ROTATION_SEQUENCE::E_SEQ_XYZ : E_ROTATION_SEQUENCE::E_SEQ_ZYX;

			cv::Mat rotMat = CTransformation_CV::rotDegree2Matrix(rx, ry, rz, rotSeq);
			cv::Vec3d angles = CTransformation_CV::rotMatrix2RotDegree(rotMat, rotSeq);

			double err = std::max(std::max(std::abs(angles[0] - rx), std::abs(angles[1] - ry)), std::abs(angles[2] - rz));
			maxErr = std::max(maxErr, err);
		}
	}

	info("最大角度误差(度)", maxErr);
	CHECK(maxErr < 1e-8);
}

static void test_cv_gimbal_lock()
{
	section("CV: 万向锁 ry = ±90");

	for (int s = 0; s < 2; ++s)
	{
		E_ROTATION_SEQUENCE rotSeq = (s == 0) ? E_ROTATION_SEQUENCE::E_SEQ_XYZ : E_ROTATION_SEQUENCE::E_SEQ_ZYX;

		for (int t = 0; t < 2; ++t)
		{
			double ry = (t == 0) ? 90.0 : -90.0;

			cv::Mat rotMat = CTransformation_CV::rotDegree2Matrix(30.0, ry, 45.0, rotSeq);
			cv::Vec3d angles = CTransformation_CV::rotMatrix2RotDegree(rotMat, rotSeq);
			cv::Mat rebuild = CTransformation_CV::rotDegree2Matrix(angles[0], angles[1], angles[2], rotSeq);

			CHECK(cv::norm(rebuild - rotMat) < 1e-12);
			CHECK(std::abs(angles[1] - ry) < 1e-9);
		}
	}
}

static void test_cv_pose_roundtrip()
{
	section("CV: 位姿 -> 齐次矩阵 -> 位姿");

	double maxErr = 0.0;
	for (int i = 0; i < 1000; ++i)
	{
		S_POSE in(uniform(-1000.0, 1000.0), uniform(-1000.0, 1000.0), uniform(-1000.0, 1000.0),
			uniform(-180.0, 180.0), uniform(-80.0, 80.0), uniform(-180.0, 180.0));

		cv::Mat hmMatrix = CTransformation_CV::pose2HmMatrix(in);
		S_POSE out = CTransformation_CV::hmMatrix2Pose(hmMatrix);

		double err = std::max(std::max(std::abs(out.X - in.X), std::abs(out.Y - in.Y)), std::abs(out.Z - in.Z));
		err = std::max(err, std::max(std::max(std::abs(out.Rx - in.Rx), std::abs(out.Ry - in.Ry)), std::abs(out.Rz - in.Rz)));
		maxErr = std::max(maxErr, err);
	}

	info("最大位姿误差", maxErr);
	CHECK(maxErr < 1e-8);
}

static void test_cv_rodrigues_input_types()
{
	section("CV: Rodrigues 逆变换的输入类型（回归）");

	cv::Mat rotMat64 = (cv::Mat_<double>(3, 3) <<
		0.0, -1.0, 0.0,
		1.0, 0.0, 0.0,
		0.0, 0.0, 1.0);						// 绕 Z 轴 90 度
	cv::Mat rotMat32;
	rotMat64.convertTo(rotMat32, CV_32F);

	// CV_64F：正常路径
	cv::Vec3d vec64;
	CHECK(CTransformation_CV::rotMatrix2RotRadian_Rodrigues(rotMat64, vec64));
	CHECK(std::abs(vec64[0]) < 1e-12 && std::abs(vec64[1]) < 1e-12);
	CHECK(std::abs(vec64[2] - M_PI / 2) < 1e-12);

	// CV_32F：曾经因 at<double> 读 float 数据而抛异常
	bool threw32 = false;
	cv::Vec3d vec32;
	bool ok32 = false;
	try
	{
		ok32 = CTransformation_CV::rotMatrix2RotRadian_Rodrigues(rotMat32, vec32);
	}
	catch (const cv::Exception&)
	{
		threw32 = true;
	}
	CHECK(!threw32);
	CHECK(ok32);
	CHECK(cv::norm(vec32 - vec64) < 1e-6);

	// 4x4：曾经直接抛 cv::Exception
	bool threw44 = false;
	bool ok44 = true;
	cv::Vec3d vec44;
	try
	{
		ok44 = CTransformation_CV::rotMatrix2RotRadian_Rodrigues(cv::Mat::eye(4, 4, CV_64F), vec44);
	}
	catch (const cv::Exception&)
	{
		threw44 = true;
	}
	CHECK(!threw44);
	CHECK(!ok44);

	// 非旋转矩阵应被判为非法
	cv::Vec3d vecBad;
	CHECK(!CTransformation_CV::rotMatrix2RotRadian_Rodrigues(1.00001 * cv::Mat::eye(3, 3, CV_64F), vecBad));
	CHECK(!CTransformation_CV::rotMatrix2RotDegree_Rodrigues(cv::Mat::eye(4, 4, CV_64F), vecBad));
}

static void test_cv_rodrigues_roundtrip()
{
	section("CV: Rodrigues 往返");

	double maxErr = 0.0;
	int rejected = 0;
	for (int i = 0; i < 2000; ++i)
	{
		double theta = uniform(-(M_PI - 0.1), (M_PI - 0.1));
		Eigen::Vector3d unit = randomUnitVector();
		Eigen::Vector3d rotVec = theta * unit;

		cv::Mat rotMat = CTransformation_CV::rotRadian2Matrix_Rodrigues(rotVec.x(), rotVec.y(), rotVec.z());

		cv::Vec3d back;
		if (!CTransformation_CV::rotMatrix2RotRadian_Rodrigues(rotMat, back))
		{
			++rejected;
			continue;
		}
		double err = std::sqrt((back[0] - rotVec.x()) * (back[0] - rotVec.x())
			+ (back[1] - rotVec.y()) * (back[1] - rotVec.y())
			+ (back[2] - rotVec.z()) * (back[2] - rotVec.z()));
		maxErr = std::max(maxErr, err);
	}

	info("最大旋转向量误差(rad)", maxErr);
	CHECK(rejected == 0);
	CHECK(maxErr < 1e-9);
}

static void test_cv_rodrigues_pose_angle_type()
{
	section("CV: S_POSE 版 Rodrigues 的角度单位透传");

	S_POSE poseRadian(1.0, 2.0, 3.0, 0.0, 0.0, M_PI / 2);
	S_POSE poseDegree(1.0, 2.0, 3.0, 0.0, 0.0, 90.0);

	cv::Mat byRadian = CTransformation_CV::pose2HmMatrix_Rodrigues(poseRadian, E_ANGLE_TYPE::E_TYPE_RADIAN);
	cv::Mat byDegree = CTransformation_CV::pose2HmMatrix_Rodrigues(poseDegree, E_ANGLE_TYPE::E_TYPE_DEGREE);

	CHECK(cv::norm(byRadian - byDegree) < 1e-12);
	CHECK(std::abs(byRadian.at<double>(0, 1) + 1.0) < 1e-12);
	CHECK(std::abs(byRadian.at<double>(1, 0) - 1.0) < 1e-12);
}

// ===========================================================================
// 两套实现的一致性
// ===========================================================================
static void test_eg_cv_consistency()
{
	section("EG 与 CV 结果一致性");

	double maxPoseErr = 0.0;
	double maxMatrixErr = 0.0;
	for (int i = 0; i < 1000; ++i)
	{
		double x = uniform(-1000.0, 1000.0);
		double y = uniform(-1000.0, 1000.0);
		double z = uniform(-1000.0, 1000.0);
		double rx = uniform(-180.0, 180.0);
		double ry = uniform(-80.0, 80.0);
		double rz = uniform(-180.0, 180.0);

		S_POSE pose(x, y, z, rx, ry, rz);

		for (int s = 0; s < 2; ++s)
		{
			E_ROTATION_SEQUENCE rotSeq = (s == 0) ? E_ROTATION_SEQUENCE::E_SEQ_XYZ : E_ROTATION_SEQUENCE::E_SEQ_ZYX;

			Eigen::Matrix4d egMatrix = CTransformation_EG::pose2HmMatrix(pose, rotSeq);
			cv::Mat cvMatrix = CTransformation_CV::pose2HmMatrix(pose, rotSeq);

			for (int r = 0; r < 4; ++r)
			{
				for (int c = 0; c < 4; ++c)
				{
					maxMatrixErr = std::max(maxMatrixErr, std::abs(egMatrix(r, c) - cvMatrix.at<double>(r, c)));
				}
			}

			S_POSE egPose = CTransformation_EG::hmMatrix2Pose(egMatrix, rotSeq);
			S_POSE cvPose = CTransformation_CV::hmMatrix2Pose(cvMatrix, rotSeq);

			double err = std::max(std::max(std::abs(egPose.X - cvPose.X), std::abs(egPose.Y - cvPose.Y)), std::abs(egPose.Z - cvPose.Z));
			err = std::max(err, std::max(std::max(std::abs(egPose.Rx - cvPose.Rx), std::abs(egPose.Ry - cvPose.Ry)), std::abs(egPose.Rz - cvPose.Rz)));
			maxPoseErr = std::max(maxPoseErr, err);
		}
	}

	info("齐次矩阵最大差异", maxMatrixErr);
	info("位姿最大差异", maxPoseErr);
	CHECK(maxMatrixErr < 1e-9);
	CHECK(maxPoseErr < 1e-6);
}

// ===========================================================================
int main()
{
	std::cout << std::setprecision(6) << std::boolalpha;
	std::cout << "================ 位姿变换回归测试 ================" << std::endl;

	test_tolerance_constants();

	test_eg_angle_unit();
	test_eg_euler_roundtrip(E_ROTATION_SEQUENCE::E_SEQ_XYZ, "EG: XYZ 欧拉角往返");
	test_eg_euler_roundtrip(E_ROTATION_SEQUENCE::E_SEQ_ZYX, "EG: ZYX 欧拉角往返");
	test_eg_gimbal_lock();
	test_eg_pose_roundtrip();
	test_eg_rodrigues_roundtrip();
	test_eg_rodrigues_near_pi();
	test_eg_small_angle();
	test_eg_nan_vanish();
	test_eg_invalid_input();
	test_eg_rodrigues_pose_angle_type();

	test_cv_euler_roundtrip();
	test_cv_gimbal_lock();
	test_cv_pose_roundtrip();
	test_cv_rodrigues_input_types();
	test_cv_rodrigues_roundtrip();
	test_cv_rodrigues_pose_angle_type();

	test_eg_cv_consistency();

	std::cout << std::endl << "================ 测试结果 ================" << std::endl;
	std::cout << "  用例总数: " << g_checked << std::endl;
	std::cout << "  失败个数: " << g_failed << std::endl;

	if (g_failed == 0)
	{
		std::cout << "  结论: 全部通过" << std::endl;
	}
	else
	{
		std::cout << "  结论: 存在失败用例" << std::endl;
	}

	return g_failed;
}
