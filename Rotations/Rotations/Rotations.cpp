#include "Rotations.h"
#include <cmath>
#include <stdexcept>
#include <iostream>

#define PI 3.14
#define EPS 1e-7

//_______________________________________________________________________________________________________________________//
// 														GENERALS													     //
//_______________________________________________________________________________________________________________________//

//To changge the angle to radians
double Rotation::toRadians(double angle)  {
    return angle * PI / 180.0;
}

//To change the radiantrs to degrees
double Rotation::toDegrees(double angle)  {
    return angle * 180.0 / PI;
}

//To checck if the determinant is correct
double Rotation::Determinant(std::vector<std::vector<double>>& matrix) {

    return matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
        matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
        matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
}

//Check if is a rotation matrix with the transpose and the inverse, i didnt actually make this one
bool Rotation::TransposeInvese( std::vector<std::vector<double>>& matrix) {

    // Calculate transpose
    std::vector<std::vector<double>> transpose = {
        {matrix[0][0], matrix[1][0], matrix[2][0]},
        {matrix[0][1], matrix[1][1], matrix[2][1]},
        {matrix[0][2], matrix[1][2], matrix[2][2]}
    };

    // Multiply 
    std::vector<std::vector<double>> product(3, std::vector<double>(3, 0.0));
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                product[i][j] += matrix[i][k] * transpose[k][j];
            }
        }
    }

    // Check if product is an identity matrix
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (i == j && std::abs(product[i][j] - 1.0) > 1e-9) {
                return false;
            }
            else if (i != j && std::abs(product[i][j]) > 1e-9) {
                return false;
            }
        }
    }

    return true;
}

//Print the matrix
void Rotation::printMatrix(const std::vector<std::vector<double>>& matrix) {

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (abs(matrix[i][j]) > EPS)
            {
                std::cout << matrix[i][j] << " ";
            }
            else
            {
                std::cout << 0 << " ";
            }
        }
        std::cout << std::endl; // Move to the next line after each row
    }
}

//_______________________________________________________________________________________________________________________//
// 														TO DO 1 													     //
//_______________________________________________________________________________________________________________________//

//To convert the euler notatioon to a rotation matrix
std::vector<std::vector<double>> Rotation::eulerToRMatrix( std::vector<double>& axis, double angle) {

    //Put an easyear nomenclature
    double x = axis[0];
    double y = axis[1];
    double z = axis[2];

	// Get the norm, also to check if the axis is not 0
    double norm = std::sqrt(x * x + y * y + z * z);

    if (norm != 1) {
		std::cout << "Axis norm is == 0." << std::endl;
		return { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };
    }

	double radians = toRadians(angle);
    double cos = std::cos(radians);
    double sin = std::sin(radians);
    double tan = 1 - cos;

    return {
        {tan * x * x + cos, tan * x * y - z * sin, tan * x * z + y * sin},
        {tan * x * y + z * sin, tan * y * y + cos, tan * y * z - x * sin},
        {tan * x * z - y * sin, tan * y * z + x * sin, tan * z * z + sin}
    };
}

//_______________________________________________________________________________________________________________________//
// 														TO DO 2 													     //
//_______________________________________________________________________________________________________________________//

//Quat multiplication
std::vector<double> Rotation::quaternionMultiply(const std::vector<double>& q1, const std::vector<double>& q2) {
    if (q1.size() != 4 || q2.size() != 4) {
        throw std::invalid_argument("Quaternions must have 4 components.");
    }

	//The multiplication (the looooong way) from class
    return {
        q1[3] * q2[0] + q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1],
        q1[3] * q2[1] - q1[0] * q2[2] + q1[1] * q2[3] + q1[2] * q2[0],
        q1[3] * q2[2] + q1[0] * q2[1] - q1[1] * q2[0] + q1[2] * q2[3],
        q1[3] * q2[3] - q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2]
    };
}

//The rotation
std::vector<double> Rotation::rotateVectorWithQuaternion( std::vector<double>& vec,  std::vector<double>& quat) {

    if (vec.size() != 3 || quat.size() != 4) {
        throw std::invalid_argument("Vector must have 3 components and quaternion must have 4 components.");
    }

	//the vector in a quaternion form
    std::vector<double> vecQuat = { vec[0], vec[1], vec[2], 0.0 };
	//The conjugate of the quaternion
    std::vector<double> quatConjugate = { -quat[0], -quat[1], -quat[2], quat[3] };

    //Compute the result using the multiplication
    std::vector<double> rotatedQuat = quaternionMultiply(quaternionMultiply(quat, vecQuat), quatConjugate);

    return { rotatedQuat[0], rotatedQuat[1], rotatedQuat[2] };
}

//_______________________________________________________________________________________________________________________//
// 														TO DO 3 													     //
//_______________________________________________________________________________________________________________________//

void Rotation::Traces() {

    std::cout << "Angle,Trace\n";

    std::vector<double> axis = { 0, 0, 1 }; // Z axis as eulers
    double maxAngle = 6 * PI;

    for (int i = 0; i <= 100; ++i) {

		// we are multypling the max angle with values under 1, so its basically deviding till we get to the max value
        double angle = maxAngle * i / 100;

        //L'ets use what we have :D
        Rotation rotation;
        auto matrix = rotation.eulerToRMatrix(axis, angle * 180 / PI);

        //Trace
        double trace = matrix[0][0] + matrix[1][1] + matrix[2][2];

        std::cout <<"Angle:  " << angle << "\n" << "Trace:  " << trace << "\n\n";
    }
}

void main() {
	Rotation rotation;
    
	//Set the axis
	std::vector<double> axis = { 0, 0, 1 }; // Z axis as eulers

    //Do the rotation
    std::vector<std::vector<double>> matrix = rotation.eulerToRMatrix(axis, 90);

    //Sheck if its actrually a rotation matrice
    if (rotation.Determinant(matrix) == 0)
    {
		std::cout << "The matrix is not a rotation matrix." << std::endl;
    }
    if (rotation.TransposeInvese(matrix) == false)
    {
		std::cout << "The matrix is not a rotation matrix." << std::endl;
    }


	//Print the matrix
    rotation.printMatrix(matrix);
	rotation.Traces();
}


