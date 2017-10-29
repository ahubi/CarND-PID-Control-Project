#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID()
: p_error(0)
, i_error(0)
, d_error(0)
, Kp(0)
, Ki(0)
, Kd(0)
{}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
}

void PID::UpdateError(double cte) {
}

double PID::TotalError() {
  double ret = 0;
  return ret;
}

