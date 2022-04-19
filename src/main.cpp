#include "robot.h"
#include "estimator.h"
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <vector>

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
  int map_index = -1;


  //Place robot in map
  mapspace.segment(0, R.size()) = Eigen::RowVectorXi::Ones(R.size());
  x.segment(0, R.size()) = R;
  P.block(0, 0, R.size(), R.size()) = Eigen::MatrixXf::Zero(R.size(), R.size());
  map_index += R.size();


  for (int t = 0; t < 20; ++t)
  {
    //SIMULATOR
    Eigen::Vector2f n = q.array() * Eigen::VectorXf::Random(2).array();

    robot.move(U, Eigen::VectorXf::Zero(2));

    for (int i = 0; i < N; ++i)
    {
      Eigen::Vector2f v = s.array() * Eigen::VectorXf::Random(2).array();
      Eigen::Matrix<float, 2, 6> Y_temp = robot.observe(robot.get_pose(), W.col(i));
      Y.col(i) = Y_temp.block<2, 1>(0, 0) + v;
    }


    // //ETSIMATOR
    //a. Landmark Initialization
    std::vector<Eigen::Index> land_indexs;
    for (Eigen::Index i = 0; i < landmarks.row(0).size(); ++i)
    {
      Eigen::Vector2i landmark_i = landmarks.col(i);
      if (landmark_i(0) == 0 && landmark_i(1) == 0) {
        land_indexs.push_back(i);
      }
    }

    if (land_indexs.size() > 0) {
      int i = rand() % (land_indexs.size() - 1);
      if (map_index <= mapspace.size() - 1 - W.rows()) {
        map_index += W.rows();
        landmarks(0, i) = map_index - 1;
        landmarks(1, i) = map_index;
        mapspace.segment(landmarks(0, i), W.rows()) = Eigen::RowVectorXi::Ones(W.rows());

        Eigen::Vector2f Yi = Y.col(i);

        Eigen::Matrix<float, 2, 6> xl_with_J = robot.inv_observe(x.segment(0, R.size()), Yi);
        Eigen::Matrix<float, 2, 3> L_r = xl_with_J.block<2, 3>(0, 1);
        Eigen::Matrix<float, 2, 2> L_y = xl_with_J.block<2, 2>(0, 4);

        x.segment(landmarks(0, i), W.rows()) = xl_with_J.block<2, 1>(0, 0);
        P.block(map_index - 1, 0, W.rows(), map_index - 1) = 
          L_r * P.block(0, 0, R.size(), map_index - 1);
        P.block(0, map_index - 1, map_index - 1, W.rows()) = 
          P.block(map_index - 1, 0, W.rows(), map_index - 1).transpose();
        P.block(map_index - 1, map_index - 1, W.rows(), W.rows()) = 
          L_r * P.block(0, 0, R.size(), R.size()) * L_r.transpose() + 
          L_y * S * L_y.transpose();
      }
    }
    


    //b. Prediction -- robot motion
    Eigen::Matrix<float, 3, 6> xr_with_J = robot.move(x.segment(0, R.size()), U, n);
    Eigen::Matrix<float, 3, 3> R_r = xr_with_J.block<3, 3>(0, 1);
    Eigen::Matrix<float, 3, 2> R_n = xr_with_J.block<3, 2>(0, 4);

    x.segment(0, R.size()) = xr_with_J.block<3, 1>(0, 0);  
    P.block(0, R.size(), R.size(), map_index + 1 - R.size()) = 
      R_r * P.block(0, R.size(), R.size(), map_index + 1 - R.size());
    P.block(R.size(), 0, map_index + 1 - R.size(), R.size()) = 
      P.block(0, R.size(), R.size(), map_index + 1 - R.size()).transpose();
    P.block(0, 0, R.size(), R.size()) = 
      R_r * P.block(0, 0, R.size(), R.size()) * R_r.transpose() + 
      R_n * Q * R_n.transpose();

    std::cout << x.transpose() << std::endl << std::endl;
  }

}