#include <Eigen/Core>
#include <iostream>
#include <cmath>

#include "robot.h"
#include "frame.h"



Eigen::Vector3f Robot::move(Eigen::Vector2f u, Eigen::Vector2f n)
{
  float a = pose(2);
  float dx = u(0) + n(0);
  float da = u(1) + n(1);
  float ao = a + da;

  if (ao > M_PI) {
    ao = ao - 2 * M_PI;
  }
  if (ao < M_PI) {
    ao = ao + 2 * M_PI;
  }

  Eigen::Vector2f dp(dx, 0);
  Eigen::Matrix<float, 2, 6> to = from_frame(pose, dp);

  pose.head(2) = to.block<2, 1>(0, 0);
  pose(2) = ao;
}





Eigen::Matrix<float, 3, 6> Robot::move(Eigen::Vector3f x, Eigen::Vector2f u, Eigen::Vector2f n)
{
  float a = x(2);
  float dx = u(0) + n(0);
  float da = u(1) + n(1);
  float ao = a + da;

  if (ao > M_PI) {
    ao = ao - 2 * M_PI;
  }
  if (ao < M_PI) {
    ao = ao + 2 * M_PI;
  }

  Eigen::Vector2f dp(dx, 0);
  Eigen::Matrix<float, 2, 6> to = from_frame(x, dp);

  //updated x in global frame
  Eigen::Vector3f ro = Eigen::Vector3f::Zero();
  ro.head(2) = to.block<2, 1>(0, 0);
  ro(2) = ao;

  Eigen::Matrix<float, 3, 6> jacobian = Eigen::Matrix<float, 3, 6>::Zero();
  jacobian.block<3, 1>(0, 0) = ro;
  jacobian.block<2, 3>(0, 1) = to.block<2, 3>(0, 1);
  jacobian(2, 3) = 1;
  jacobian.block<2, 1>(0, 4) = to.block<2, 1>(0, 4);
  jacobian(2, 5) = 1;

  return jacobian;
}



Eigen::Vector3f Robot::get_pose()
{
  return pose;
}

void Robot::print()
{
  std::cout << pose << std::endl;
}




Eigen::Matrix<float, 2, 6> Robot::observe(Eigen::Vector3f r, Eigen::Vector2f p)
{
  Eigen::Matrix<float, 2, 6> pr = to_frame(r, p);
  Eigen::Matrix<float, 2, 3> y = scan(pr.block<2, 1>(0, 0));

  Eigen::Matrix<float, 2, 6> y_out = Eigen::Matrix<float, 2, 6>::Zero();
  y_out.block<2, 1>(0, 0) = y.block<2, 1>(0, 0);
  y_out.block<2, 3>(0, 1) = y.block<2, 2>(0, 1) * pr.block<2, 3>(0, 1);
  y_out.block<2, 2>(0, 4) = y.block<2, 2>(0, 1) * pr.block<2, 2>(0, 4);

  return y_out;
}




Eigen::Matrix<float, 2, 6> Robot::inv_observe(Eigen::Vector3f r, Eigen::Vector2f y)
{
  Eigen::Matrix<float, 2, 3> pr = inv_scan(y);
  Eigen::Matrix<float, 2, 6> p = from_frame(r, pr.block<2, 1>(0, 0));

  Eigen::Matrix<float, 2, 6> p_out = Eigen::Matrix<float, 2, 6>::Zero();
  p_out.block<2, 1>(0, 0) = p.block<2, 1>(0, 0);
  p_out.block<2, 3>(0, 1) = p.block<2, 3>(0, 1);
  p_out.block<2, 2>(0, 4) = p.block<2, 2>(0, 4) * pr.block<2, 2>(0, 1);  
  return p_out;
}




Eigen::Matrix<float, 2, 3> Robot::scan(Eigen::Vector2f p)
{
  float px = p(0);
  float py = p(1);

  float d = sqrt(px * px + py * py);
  float a = atan2(py, px);

  Eigen::Matrix<float, 2, 3> Y_p {{d, px / (float)sqrt(px * px + py * py), py / (float)sqrt(px * px + py * py)},
    {a, -py / (px * px * (py * py / px / px + 1)), 1 / (px * (py * py / px / px + 1))}};
  return Y_p;
}


Eigen::Matrix<float, 2, 3> Robot::inv_scan(Eigen::Vector2f y)
{
  float d = y(0);
  float a = y(1);

  float px = d * cosf(a);
  float py = d * sinf(a);
  
  Eigen::Matrix<float, 2, 3> P_y{{px, cosf(a), -d * sinf(a)}, 
    {py, sinf(a), d * cosf(a)}};
  return P_y;
}


