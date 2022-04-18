#include <Eigen/Core>

Eigen::Matrix<float, 2, 6> to_frame(Eigen::Vector3f F, Eigen::Vector2f pw);
Eigen::Matrix<float, 2, 6> from_frame(Eigen::Vector3f F, Eigen::Vector2f pf);