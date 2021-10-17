#ifndef LTI_H
#define LTI_H
#include <Eigen/Dense>

#pragma once

class LTISystem{

public:
 LTISystem(
      const Eigen::MatrixXd& A,
      const Eigen::MatrixXd& B,
      const Eigen::MatrixXd& C
  );


  void init(const Eigen::VectorXd& x0);

  void update(const Eigen::VectorXd& u);

  Eigen::VectorXd state() { return stateX; };
  Eigen::VectorXd output();

private:

  Eigen::MatrixXd A, B, C;

  int m, n;
  bool initialized;

  Eigen::VectorXd stateX;
  Eigen::VectorXd outY;
};
#endif // LTI_H
