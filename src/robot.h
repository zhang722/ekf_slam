#include <Eigen/Core>

class Robot
{
public:
  Robot(Eigen::Vector3f pose) : pose(pose) {}
  ~Robot() {};

  //robot
  void move(Eigen::Vector2f u, Eigen::Vector2f n);
  Eigen::Matrix<float, 3, 6> move(Eigen::Vector3f x, Eigen::Vector2f u, Eigen::Vector2f n);

  Eigen::Vector3f get_pose();
  void print();
  Eigen::Matrix<float, 2, 6> observe(Eigen::Vector3f r, Eigen::Vector2f p);
  Eigen::Matrix<float, 2, 6> inv_observe(Eigen::Vector3f r, Eigen::Vector2f y);

  //sensor
  Eigen::Matrix<float, 2, 3> scan(Eigen::Vector2f p);
  Eigen::Matrix<float, 2, 3> inv_scan(Eigen::Vector2f y);

private:
  Eigen::Vector3f pose;
};

