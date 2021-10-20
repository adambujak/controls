#include <Eigen/Dense>
#include "LTI.h"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <fstream>

#define H_VALUE      0.01
#define KP_VALUE     90
#define MASS         1
#define GRAVITY      10
#define F_APPLIED    5
#define GOAL_HEIGHT  5
#define START_HEIGHT 10
#define START_SPEED  1

class SimWriter {

private:
  std::fstream outstream;

public:
  SimWriter(void)
  {
    this->outstream.open("output.dat", std::ios_base::app);
  }

  void write(float time, float y, float v, float a, float fa)
  {
	  this->outstream << time << " " << y << " " << v << " " << a << " " << fa << std::endl;
  }

  ~SimWriter(void)
  {
    this->outstream.close();
  }
};

float get_force(float error)
{
  return fmax(KP_VALUE*error, 0);
}

void simulate_ball(void)
{
  Eigen::MatrixXd A(2, 2);
  Eigen::MatrixXd B(2, 1);
  Eigen::MatrixXd C(1, 2);
  Eigen::MatrixXd x(2, 1);

  A << 1,H_VALUE, 0,1;
  B << 0,H_VALUE;
  C << 1,0;
  x << START_HEIGHT,START_SPEED;

  LTISystem lti(A, B, C);
  lti.init(x);

  SimWriter writer;

  Eigen::VectorXd u(1, 1);
  u[0] = (get_force(0)/MASS - GRAVITY);

  Eigen::VectorXd y(1, 1);
  Eigen::VectorXd state(2, 1);

  float error;
  for (int i = 0; i < 1000; i++) {
    state = lti.state();
    lti.update(u);
    y = lti.output();

    error = GOAL_HEIGHT - y[0];
    u[0] = (get_force(error)/MASS - GRAVITY);
    std::cout << error << "," << u[0] << std::endl;

    writer.write(H_VALUE*i, y[0], state[1], 0, 0);

    if (y[0] <= 0) {
      return;
    }
  }
}

int main()
{

  std::cout << "started" << std::endl;
  simulate_ball();

  return 0;
}
