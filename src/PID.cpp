#include "PID.h"

using namespace std;

PID::PID()
: pe_(0)
, ie_(0)
, de_(0)
, Kp_(0)
, Ki_(0)
, Kd_(0)
{}

PID::~PID() {}

void PID::Init(const double& kp, const double& ki, const double& kd) {
  Kp_ = kp;
  Ki_ = ki;
  Kd_ = kd;
}

void PID::UpdateError(double cte) {
  de_  = cte - pe_;
  pe_  = cte;
  ie_ += cte;
}

double PID::TotalError() {
  double ret = -Kp_ * pe_ - Kd_ * de_ - Ki_ * ie_;
  //Normalize to [-1,1]
  while (ret > 1) ret -=1;
  while (ret < -1) ret +=1;
  return ret;
}

