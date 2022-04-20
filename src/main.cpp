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
  Eigen::Matrix<float, 2, 2> S = Eigen::Matrix<float, 2, 2>::Identity();
  S.row(0).array() *= s(0) * s(0);
  S.row(1).array() *= s(1) * s(1);
  

  Eigen::RowVectorXi mapspace = Eigen::RowVectorXi::Zero(x.size());
  Eigen::MatrixXi landmarks = Eigen::MatrixXi::Zero(2, N);
  int map_index = -1;


  //Place robot in map
  mapspace.segment(0, R.size()) = Eigen::RowVectorXi::Ones(R.size());
  x.segment(0, R.size()) = R;
  P.block(0, 0, R.size(), R.size()) = Eigen::MatrixXf::Zero(R.size(), R.size());
  map_index += R.size();


  for (int t = 0; t < 50; ++t)
  {
    //SIMULATOR
    Eigen::Vector2f n = q.array() * Eigen::VectorXf::Random(2).array();

    robot.move(U, Eigen::VectorXf::Zero(2));

    for (int i = 0; i < N; ++i)
    {
      Eigen::Vector2f v = s.array() * Eigen::VectorXf::Random(2).array();
      Eigen::Matrix<float, 2, 6> Y_temp = robot.observe(robot.get_pose(), W.col(i));
      Y.col(i) = Y_temp.block<2, 1>(0, 0);
    }


    // //ETSIMATOR

    //b. Prediction -- robot motion
    Eigen::Matrix<float, 3, 6> xr_with_J = robot.move(x.segment(0, R.size()), U, Eigen::VectorXf::Zero(2));
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



    //c. Landmark Correction -- known landmarks
    std::vector<Eigen::Index> land_indexs;
    land_indexs.clear();
    for (Eigen::Index i = 0; i < landmarks.cols(); ++i)
    {
      Eigen::Vector2i landmark_i = landmarks.col(i);
      if (!(landmark_i(0) == 0 && landmark_i(1) == 0)) {
        land_indexs.push_back(i);
      }
    }


    for (std::vector<Eigen::Index>::iterator it = land_indexs.begin(); it != land_indexs.end(); ++it) {


      Eigen::Vector2i landmark_i = landmarks.col(*it);
      Eigen::Matrix<float, 2, 6> hx_with_J = robot.observe(x.segment(0, R.size()), x.segment(landmark_i(0), W.rows()));

      Eigen::Matrix<float, 2, 3> E_r = hx_with_J.block<2, 3>(0, 1);
      Eigen::Matrix<float, 2, 2> E_l = hx_with_J.block<2, 2>(0, 4);

      Eigen::Matrix<float, 2, 5> E_rl;
      E_rl.block<2, 3>(0, 0) = E_r;
      E_rl.block<2, 2>(0, 3) = E_l;

      Eigen::MatrixXf P_rl_rl(5, 5);
      P_rl_rl.block<3, 3>(0, 0) = P.block<3, 3>(0, 0);
      P_rl_rl.block<3, 2>(0, 3) = P.block<3, 2>(0, landmark_i(0));
      P_rl_rl.block<2, 3>(3, 0) = P.block<2, 3>(landmark_i(0), 0);
      P_rl_rl.block<2, 2>(3, 3) = P.block<2, 2>(landmark_i(0), landmark_i(0));

      Eigen::Matrix<float, 2, 2> E = E_rl * P_rl_rl * E_rl.transpose(); 

      Eigen::Vector2f Yi = Y.col(*it);
      Eigen::Vector2f z = Yi - hx_with_J.block<2, 1>(0, 0);


      if (z(1) > M_PI) {
        z(1) -= 2 * M_PI;
      }
      if (z(1) < -M_PI) {
        z(1) += 2 * M_PI;
      }
 
      Eigen::Matrix<float, 2, 2> Z = S + E;
      if (z.transpose() * Z.inverse() * z < 9) {
        Eigen::MatrixXf P_rm_rl(landmark_i(1) + 1, 5);
        P_rm_rl.block(0, 0, landmark_i(1) + 1, 3) = P.block(0, 0, landmark_i(1) + 1, 3);
        P_rm_rl.block(0, 3, landmark_i(1) + 1, 2) = P.block(0, landmark_i(0), landmark_i(1) + 1, 2);
        Eigen::MatrixXf K(landmark_i(1) + 1, 2);
        K = P_rm_rl * E_rl.transpose() * Z.inverse();

// if (t == 22 || t == 23) {
//   std::cout << "x:" << std::endl << x.transpose() << std::endl;
//   std::cout << "z:" << std::endl << z.transpose() << std::endl;
//   std::cout << "K:" << std::endl << K << std::endl;
//   std::cout << "K*z:" << std::endl << (K*z).transpose() << std::endl;
// }
        x.segment(0, landmark_i(1) + 1) += K * z;
        P.block(0, 0, landmark_i(1) + 1, landmark_i(1) + 1) -= K * Z * K.transpose(); 
      }


    }

    //a. Landmark Initialization
    land_indexs.clear();
    for (Eigen::Index i = 0; i < landmarks.cols(); ++i)
    {
      Eigen::Vector2i landmark_i = landmarks.col(i);
      if (landmark_i(0) == 0 && landmark_i(1) == 0) {
        land_indexs.push_back(i);
      }
    }

    for (int j = 0; j < mapspace.size() && mapspace(j) == 1; ++j)
    {
      map_index = j;
    }

    if (land_indexs.size() > 0) {
      int rand_index = (rand() % land_indexs.size() + 1) - 1;
      Eigen::Index i = land_indexs[rand_index];

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
    std::cout << t << " " << map_index;
    std::cout << "robot:" << robot.get_pose().transpose() << " x_r: " << x.segment(0, 3).transpose() << std::endl;
    std::cout << "P_ll " << P.block(map_index - 1, map_index - 1, W.rows(), W.rows()) << std::endl;

  }

}