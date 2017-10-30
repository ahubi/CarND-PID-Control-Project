#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID()
: p_error_(0)
, i_error_(0)
, d_error_(0)
, koeff_p_(0)
, koeff_i_(0)
, koeff_d_(0)
{}

PID::~PID() {}

void PID::Init(const double& kp, const double& ki, const double& kd) {
  koeff_p_ = kp;
  koeff_i_ = ki;
  koeff_d_ = kd;
}

void PID::UpdateError(double cte) {
  d_error_  = cte - p_error_;
  p_error_  = cte;
  i_error_ += cte;
}

double PID::TotalError() {
  double ret = -koeff_p_ * p_error_ - koeff_d_ * d_error_ - koeff_i_ * i_error_;
  while (ret > 1) ret -=1;
  while (ret < -1) ret +=1;
  return ret;
}

