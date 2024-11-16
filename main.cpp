#include <iostream>
#include <string>

#include "transformation_eg.h"

using namespace std;

int main()
{
	double x, y, z;
	x = 300;
	y = 734;
	z = 856;

	double rx, ry, rz;
	rx = -136.142;
	ry = 77.359;
	rz = 14.78;

	S_POSE pose(x, y, z, rx, ry, rz);

	std::cout << "X Y Z Rx Ry Rz = " << x << " " << y << " " << z << " " << rx << " " << ry << " " << rz << std::endl;

	Eigen::Matrix3d retMat = CTransformation::rotDegree2Matrix(rx, ry, rz/*,E_ROTATION_SEQUENCE::E_SEQ_ZYX*/);

	cout << "retMat = " << endl << retMat << endl;

	Eigen::Vector3d retVec = CTransformation::rotMatrix2RotDegree(retMat/*, E_ROTATION_SEQUENCE::E_SEQ_ZYX*/);

	cout << "retVec = " << endl << retVec << endl;

	Eigen::Matrix4d hmMatrix = CTransformation::pose2HmMatrix(pose/*, E_ROTATION_SEQUENCE::E_SEQ_ZYX*/);

	cout << "hmMatrix = " << endl << hmMatrix << endl;

	Eigen::Matrix4d handEyeMatrix;
	handEyeMatrix << -0.7823792128542812, -0.6224001076123106, -0.02238019967961234, 104.1779385005517,
	0.6221065415220007, -0.7826985336558441, 0.01914305118121669, -23.06481871230535,
	-0.0294315865873748, 0.001054256693534112, 0.9995662410534754, 129.5884080840378,
	0, 0, 0, 1;

	S_POSE poseHandEye = CTransformation::hmMatrix2Pose(handEyeMatrix, E_ROTATION_SEQUENCE::E_SEQ_ZYX);

	std::cout << "Pose = [" << poseHandEye.X << " " << poseHandEye.Y << " " << poseHandEye.Z <<
		" " << poseHandEye.Rx << " " << poseHandEye.Ry << " " << poseHandEye.Rz << "]"; 

	/*Eigen::Matrix4d hmMatrix = CTransformation::pose2HmMatrix(x, y, z, rx, ry, rz);

	std::cout << "hmMatrix = " << std::endl << hmMatrix;

	S_POSE robPose = CTransformation::hmMatrix2Pose(hmMatrix);

	std::cout << "RobPose = [" << robPose.X << " " << robPose.Y << " " << robPose.Z <<
		" " << robPose.Rx << " " << robPose.Ry << " " << robPose.Rz << "]";*/

	return 0;
}