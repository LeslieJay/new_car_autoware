#include "can_driver/motor_speed_step_encoder.hpp"

#include <cassert>
#include <limits>

int main()
{
  using can_driver::encodeMotorSpeedStep;
  using can_driver::encodeDirectionalMotorSpeedStep;
  using can_driver::encodeSignedCommand;
  using can_driver::encodePiecewiseCommand;
  using can_driver::isValidMonotonicLookupTable;

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

  const std::vector<double> speed_breakpoints{0.1, 0.5, 1.0};
  const std::vector<int64_t> speed_commands{90, 500, 1020};
  assert(encodePiecewiseCommand(0.0, speed_breakpoints, speed_commands, 4000) == 0);
  assert(encodePiecewiseCommand(0.05, speed_breakpoints, speed_commands, 4000) == 90);
  assert(encodePiecewiseCommand(0.3, speed_breakpoints, speed_commands, 4000) == 295);
  assert(encodePiecewiseCommand(2.0, speed_breakpoints, speed_commands, 4000) == 1020);
  assert(encodePiecewiseCommand(0.3, speed_breakpoints, speed_commands, 200) == 200);
  assert(isValidMonotonicLookupTable(speed_breakpoints, speed_commands));
  assert(!isValidMonotonicLookupTable({0.1, 0.1}, {1, 2}));
  assert(!isValidMonotonicLookupTable({0.1}, {1, 2}));
  assert(!isValidMonotonicLookupTable({0.1, 0.2}, {2, 1}));
  assert(!isValidMonotonicLookupTable({0.1}, {40000}));
  return 0;
}
