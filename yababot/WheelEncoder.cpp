#include "WheelEncoder.h"
#include <math.h>

WheelEncoder::WheelEncoder(double ticksPerMotorRevolution, double gearRatio, double wheelDiameterMm) {
  distancePerTick = wheelDiameterMm * M_PI / gearRatio / ticksPerMotorRevolution;
}

double WheelEncoder::getDistance(int32_t ticks) {
  return ticks * distancePerTick;
}

double WheelEncoder::getSpeed(int32_t ticks, double dt) {
  return ticks * distancePerTick / dt;
}
