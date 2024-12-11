#include <vector>

class Rotation {
public:

    Rotation() {};

    //GENERALS
    double toRadians(double angle) ;
    double toDegrees(double angle) ;
    double Determinant(std::vector<std::vector<double>>& matrix);
    bool TransposeInvese(std::vector<std::vector<double>>& matrix);
	void printMatrix(const std::vector<std::vector<double>>& matrix);

    //TODO 1
    std::vector<std::vector<double>> eulerToRMatrix( std::vector<double>& axis, double angle);

	//TODO 2
    std::vector<double> quaternionMultiply(const std::vector<double>& q1, const std::vector<double>& q2);
    std::vector<double> rotateVectorWithQuaternion( std::vector<double>& vec,  std::vector<double>& quat);

    //TODO 3
    void Traces();

};