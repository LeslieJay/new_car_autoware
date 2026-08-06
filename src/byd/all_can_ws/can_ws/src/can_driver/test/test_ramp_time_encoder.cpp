#include "can_driver/ramp_time_encoder.hpp"

#include <cassert>
#include <limits>

int main()
{
  using can_driver::encodeRampTime;

  assert(encodeRampTime(0.2, 0.0, 0.2, 10) == 10);   // 1.0 s
  assert(encodeRampTime(0.0, 0.2, -0.2, 10) == 10);  // 1.0 s
  assert(encodeRampTime(0.0, 0.2, -3.4, 10) == 1);   // clamp to 0.1 s
  assert(encodeRampTime(1.0, 0.0, 0.5, 10) == 20);   // 2.0 s
  assert(encodeRampTime(4.0, 0.0, 0.1, 10) == 255);  // clamp to 25.5 s
  assert(encodeRampTime(0.2, 0.0, 0.0, 10) == 10);
  assert(encodeRampTime(
           std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0, 10) == 10);
  return 0;
}
