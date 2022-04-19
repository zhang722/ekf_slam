#include "robot.h"
#include "estimator.h"
#include <iostream>

int main(int argc, char const *argv[])
{
  //////////////////
  Eigen::Matrix<float, 2, Eigen::Dynamic> W = cloister(-4, 4, -4, 4, 7);
  int N = W.cols();
  Robot robot(Eigen::Vector3f(0, -2, 0));
  Eigen::Vector3f R(0, -2, 0);
  Eigen::Vector2f U(0.1f, 0.05f); 
  Eigen::MatrixXf Y = Eigen::MatrixXf::Zero(2, N);

  //Estimator
  Eigen::VectorXf x = Eigen::VectorXf::Zero(R.size() + N * W.rows());
  Eigen::MatrixXf P = Eigen::MatrixXf::Zero(x.size(), x.size());

  Eigen::Vector2f q(0.01, 0.02);
  Eigen::DiagonalMatrix<float, 2> Q(q(1) * q(1), q(2) * q(2));

  Eigen::RowVectorXf mapspace = Eigen::RowVectorXf::Zero(x.size());
  Eigen::MatrixXi landmarks = Eigen::MatrixXi::Zero(2, N);
}