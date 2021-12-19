#include <Eigen/Dense>
#include "LTI.h"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <fstream>

#define H_VALUE      0.01

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

class PID {
private:
  float i_error;
  float d_error;
  float last_error;
  float kp;
  float kd;
  float ki;
  int i;

public:
  PID(float kp, float ki, float kd)
  {
    this->kp = kp;
    this->kd = kd;
    this->ki = ki;
    this->i_error = 0;
    this->d_error = 0;
    this->last_error = 0;
    this->i = 0;
  }

  void set_gains(float kp, float ki, float kd)
  {
    this->kp = kp;
    this->kd = kd;
    this->ki = ki;
  }

  float get_output(float error)
  {
    // If index 0 derivate => 0
    if (this->i == 0) {
      this->last_error = error;
    }

    this->i_error += error;
    this->d_error = (error - this->last_error) / H_VALUE;
    this->last_error = error;
    this->i++;

    return (this->kp *error) + (this->kd * this->d_error) + (this->ki * this->i_error);
  }
};

static PID pid(0,0,0);

float get_force(float error)
{
  float pid_output = pid.get_output(error);
  return pid_output;
  //return fmax(pid_output, 0);
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
    //std::cout << error << "," << u[0] << std::endl;

    writer.write(H_VALUE*i, y[0], state[1], 0, 0);

    if (y[0] <= 0) {
      return;
    }
  }
}

int main(int argc, char *argv[])
{
  std::cout << "argc: " << argc << std::endl;

  float kp = std::stof(argv[1]);
  float ki = std::stof(argv[2]);
  float kd = std::stof(argv[3]);

  std::cout << "kp: " << argv[1] << std::endl;
  std::cout << "ki: " << argv[2] << std::endl;
  std::cout << "kd: " << argv[3] << std::endl;

  pid.set_gains(kp, ki, kd);

  if (argc != 4) {
    return 1;
  }

  std::cout << "started" << std::endl;
  simulate_ball();

  return 0;
}
