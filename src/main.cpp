#include "robot.h"
#include <iostream>

int main(int argc, char const *argv[])
{
  Robot robot(Eigen::Vector3f(2, 2, 1.57));
  Eigen::Vector2f u(1.0f, 0.2f); 
  Eigen::Vector2f n = Eigen::Vector2f::Zero();
  Eigen::Matrix<float, 3, 5> jacobian = Eigen::Matrix<float, 3, 5>::Zero();
  for (int i = 0; i < 10; ++i)
  {
    jacobian = robot.move(u, n);
    robot.print();
    std::cout << jacobian << std::endl;
  }
  return 0;
}