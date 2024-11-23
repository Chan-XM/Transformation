#include <iostream>
#include <string>

#include "transformation_eg.h"
#include "transformation_cv.h"

using namespace std;

int main()
{
	double x, y, z;
	x = 300;	y = 734;	z = 856;

	double rx, ry, rz;
	rx = -136.142;	ry = 77.359;	rz = 14.78;

	S_POSE pose(x, y, z, rx, ry, rz);

	std::cout << "pose = " << x << " " << y << " " << z << " " << rx << " " << ry << " " << rz << std::endl;

	// ******************************* Test CTransformation_EG ******************************* // 
	std::cout << std::endl << "******************************* Test CTransformation_EG *******************************" << std::endl << std::endl;

	Eigen::Matrix3d retMat = CTransformation_EG::rotDegree2Matrix(rx, ry, rz);

	cout << "retMat = " << endl << retMat << endl << endl;

	Eigen::Vector3d retVec = CTransformation_EG::rotMatrix2RotDegree(retMat);

	cout << "retVec = " << endl << retVec << endl << endl;

	Eigen::Matrix4d hmMatrix = CTransformation_EG::pose2HmMatrix(pose);

	cout << "hmMatrix = " << endl << hmMatrix << endl << endl;

	Eigen::Matrix4d handEyeMatrix;
	handEyeMatrix << -0.7823792128542812, -0.6224001076123106, -0.02238019967961234, 104.1779385005517,
	0.6221065415220007, -0.7826985336558441, 0.01914305118121669, -23.06481871230535,
	-0.0294315865873748, 0.001054256693534112, 0.9995662410534754, 129.5884080840378,
	0, 0, 0, 1;

	S_POSE poseHandEye = CTransformation_EG::hmMatrix2Pose(handEyeMatrix, E_ROTATION_SEQUENCE::E_SEQ_ZYX);

	std::cout << "Pose = [" << poseHandEye.X << " " << poseHandEye.Y << " " << poseHandEye.Z <<
		" " << poseHandEye.Rx << " " << poseHandEye.Ry << " " << poseHandEye.Rz << "]" << std::endl;

	/*Eigen::Matrix4d hmMatrix = CTransformation_EG::pose2HmMatrix(x, y, z, rx, ry, rz);

	std::cout << "hmMatrix = " << std::endl << hmMatrix;

	S_POSE robPose = CTransformation_EG::hmMatrix2Pose(hmMatrix);

	std::cout << "RobPose = [" << robPose.X << " " << robPose.Y << " " << robPose.Z <<
		" " << robPose.Rx << " " << robPose.Ry << " " << robPose.Rz << "]";*/

	std::cout << std::endl << "******************************* Test CTransformation_EG *******************************" << std::endl << std::endl;

	// ******************************* Test CTransformation_CV ******************************* // 

	std::cout << std::endl << "******************************* Test CTransformation_CV *******************************" << std::endl << std::endl;

	cv::Mat retMat_cv = CTransformation_CV::rotDegree2Matrix(rx, ry, rz);

	cout << "retMat_cv = " << endl << retMat_cv << endl << endl;

	cv::Vec3d retVec_cv = CTransformation_CV::rotMatrix2RotDegree(retMat_cv);

	cout << "retVec_cv = " << endl << retVec_cv << endl << endl;

	cv::Mat hmMatrix_cv = CTransformation_CV::pose2HmMatrix(pose);

	cout << "hmMatrix_cv = " << endl << hmMatrix_cv << endl << endl;

	S_POSE pose_cv = CTransformation_CV::hmMatrix2Pose(hmMatrix_cv);

	cout << "pose_cv = [" << pose_cv.X << " " << pose_cv.Y << " " << pose_cv.Z << " "
		<< pose_cv.Rx << " " << pose_cv.Ry << " " << pose_cv.Rz << "]";

	std::cout << std::endl << "******************************* Test CTransformation_CV *******************************" << std::endl << std::endl;

	return 0;
}