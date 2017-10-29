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
}

double PID::TotalError() {
  double ret = 0;
  return ret;
}

