#include "robot.h"
#include "estimator.h"
#include <iostream>
#include <cmath>

int main(int argc, char const *argv[])
{
  //////////////////
  Eigen::Matrix<float, 2, Eigen::Dynamic> W = cloister(-4, 4, -4, 4, 7);
  int N = W.cols();
  Eigen::Vector3f R(0, -2, 0);
  Robot robot(R);


  Eigen::Vector2f U(0.1f, 0.05f); 
  Eigen::MatrixXf Y = Eigen::MatrixXf::Zero(2, N);

  //Estimator
  Eigen::VectorXf x = Eigen::VectorXf::Zero(R.size() + N * W.rows());
  Eigen::MatrixXf P = Eigen::MatrixXf::Zero(x.size(), x.size());

  Eigen::Vector2f q(0.01, 0.02);
  Eigen::DiagonalMatrix<float, 2> Q(q(0) * q(0), q(1) * q(1));

  Eigen::Vector2f s(0.1, 1 * M_PI / 180);
  Eigen::DiagonalMatrix<float, 2> S(s(0) * s(0), s(1) * s(1));

  Eigen::RowVectorXi mapspace = Eigen::RowVectorXi::Zero(x.size());
  Eigen::MatrixXi landmarks = Eigen::MatrixXi::Zero(2, N);


  //Place robot in map
  mapspace.segment(0, R.size()) = Eigen::RowVectorXi::Ones(R.size());
  x.segment(0, R.size()) = R;
  P.block(0, 0, R.size(), R.size()) = Eigen::MatrixXf::Zero(R.size(), R.size());

  for (int t = 0; t < 200; ++t)
  {
    //SIMULATOR
    Eigen::Vector2f n = q * Eigen::VectorXf::Random(2);
    robot.move(U, Eigen::VectorXf::Zero(2));

    for (int i = 0; i < N; ++i)
    {
      Eigen::Vector2f v = s * Eigen::VectorXf::Random(2);
      Eigen::Matrix<float, 2, 6> Y_temp = robot.observe(robot.get_pose(), W.col(i));
      Y.col(i) = Y_temp.block<2, 1>(0, 0) + v;
    }


    //ETSIMATOR
    
  }

}