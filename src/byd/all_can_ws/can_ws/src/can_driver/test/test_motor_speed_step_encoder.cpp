#include "can_driver/motor_speed_step_encoder.hpp"

#include <cassert>
#include <limits>

int main()
{
  using can_driver::encodeMotorSpeedStep;
  using can_driver::encodeDirectionalMotorSpeedStep;
  using can_driver::encodeSignedCommand;

  assert(encodeMotorSpeedStep(0.2, 25.0, 10, true) == 5);
  assert(encodeMotorSpeedStep(-0.4, 25.0, 10, true) == 10);
  assert(encodeMotorSpeedStep(0.01, 25.0, 10, true) == 1);
  assert(encodeMotorSpeedStep(20.0, 25.0, 10, true) == 255);
  assert(encodeMotorSpeedStep(0.2, 0.0, 10, true) == 10);
  assert(encodeMotorSpeedStep(0.2, 25.0, 10, false) == 10);
  assert(encodeMotorSpeedStep(
           std::numeric_limits<double>::quiet_NaN(), 25.0, 10, true) == 10);
  assert(encodeDirectionalMotorSpeedStep(0.2, 16.0, 10, true, true) == 3);
  assert(encodeDirectionalMotorSpeedStep(0.2, 17.25, 10, true, false) == 10);
  assert(encodeDirectionalMotorSpeedStep(-0.2, 16.0, 10, true, true) == 10);
  assert(encodeDirectionalMotorSpeedStep(-0.2, 17.25, 10, true, false) == 3);
  assert(encodeDirectionalMotorSpeedStep(0.0, 16.0, 10, true, true) == 10);

  assert(encodeSignedCommand(0.2, 1000.0, 1000.0, 0.0, 0.0, 4000) == 200);
  assert(encodeSignedCommand(-0.2, 1000.0, 900.0, 0.0, 5.0, 4000) == -185);
  assert(encodeSignedCommand(10.0, 1000.0, 1000.0, 0.0, 0.0, 4000) == 4000);
  return 0;
}
