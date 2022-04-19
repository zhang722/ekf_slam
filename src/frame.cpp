#include <math.h>
#include "frame.h"



Eigen::Matrix<float, 2, 6> to_frame(Eigen::Vector3f F, Eigen::Vector2f pw)
{
  Eigen::Vector2f t = F.head(2);
  float a = F(2);

  Eigen::Matrix2f R{{cosf(a), -sinf(a)}, {sinf(a), cosf(a)}};

  Eigen::Vector2f pf = R.transpose() * (pw - t);
  Eigen::Matrix2f PF_p = R.transpose();

  float px = pw(0);
  float py = pw(1);
  float x = t(0);
  float y = t(1);

  Eigen::Matrix<float, 2, 3> PF_f{{-cosf(a), -sinf(a), cosf(a) * (py - y) - sinf(a) * (px - x)},
    {sinf(a), -cosf(a), -cosf(a) * (px - x) - sinf(a) * (py - y)}};
  Eigen::Matrix<float, 2, 6> rsl = Eigen::Matrix<float, 2, 6>::Zero();
  rsl.block<2, 1>(0, 0) = pf;
  rsl.block<2, 3>(0, 1) = PF_f;
  rsl.block<2, 2>(0, 4) = PF_p;

  return rsl;
}





Eigen::Matrix<float, 2, 6> from_frame(Eigen::Vector3f F, Eigen::Vector2f pf)
{
  Eigen::Vector2f t = F.head(2);
  float a = F(2);

  Eigen::Matrix2f R{{cosf(a), -sinf(a)}, {sinf(a), cosf(a)}};

  Eigen::Vector2f pw = R * pf + t;

  float px = pf(0);
  float py = pf(1);
  Eigen::Matrix<float, 2, 3> PW_f{{1, 0, -py * cosf(a) - px * sinf(a)},
    {0, 1, px * cos(a) - py * sin(a)}};
  Eigen::Matrix<float, 2, 6> rsl = Eigen::Matrix<float, 2, 6>::Zero();
  rsl.block<2, 1>(0, 0) = pw;
  rsl.block<2, 3>(0, 1) = PW_f;
  rsl.block<2, 2>(0, 4) = R;

  return rsl;
}

