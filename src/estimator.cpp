#include <Eigen/Dense>
#include "estimator.h"


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"


Eigen::Matrix<float, 2, Eigen::Dynamic> cloister(float xmin, float xmax, float ymin, float ymax, int n)
{
  if (n < 5)
    n = 9;

  float x0 = (xmin + xmax) / 2;
  float y0 = (ymin + ymax) / 2;
  float hsize = xmax - xmin;
  float vsize = ymax - ymin;
  Eigen::DiagonalMatrix<float, 2> tsize(hsize, vsize);

  Eigen::RowVectorXf outer = Eigen::RowVectorXf::LinSpaced(Eigen::Sequential, n-2, -(n-3)/2, (n-3)/2);
  Eigen::RowVectorXf inner = Eigen::RowVectorXf::LinSpaced(Eigen::Sequential, n-3, -((float)n-3)/2, ((float)n-5)/2);
  
  Eigen::MatrixXf N(2, 2 * n - 5);
  N.block(0, 0, 1, n-2) = outer;
  N.block(1, 0, 1, n-2) = ((n-1) / 2 * Eigen::RowVectorXf::Ones(n - 2));
  N.block(0, n-2, 1, n-3) = inner;
  N.block(1, n-2, 1, n-3) = ((n-3) / 2 * Eigen::RowVectorXf::Ones(n - 3));

  Eigen::Matrix2f R{{0, -1}, {1, 0}};
  Eigen::MatrixXf E(2, 2 * n - 5);
  E = R * N;
  Eigen::MatrixXf points(2, N.cols() * 4);

  points.block(0, 0, 2, N.cols()) = N;
  points.block(0, N.cols(), 2, E.cols()) = E;
  points.block(0, N.cols() + E.cols(), 2, N.cols()) = -N;
  points.block(0, 2 * N.cols() + E.cols(), 2, E.cols()) = -E;

  points = (tsize * points).array() / (n - 1);
  points.row(0).array() += x0;
  points.row(1).array() += y0;

  return points;
}
