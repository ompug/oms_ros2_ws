#ifndef LEGO_LOAM_PROJECTION_MATH_H
#define LEGO_LOAM_PROJECTION_MATH_H

#include <cmath>

namespace lego_loam {

constexpr float kDegreesToRadians = 3.14159265358979323846F / 180.0F;

inline float verticalAngle(float x, float y, float z) {
  return std::atan2(z, std::sqrt(x * x + y * y));
}

inline int verticalRow(float angle_radians, float bottom_degrees,
                       float resolution_radians) {
  const float offset = -(bottom_degrees - 0.1F) * kDegreesToRadians;
  return static_cast<int>((angle_radians + offset) / resolution_radians);
}

inline bool isGroundPair(float dx, float dy, float dz,
                         float mount_angle_radians,
                         float threshold_radians = 10.0F * kDegreesToRadians) {
  const float angle = std::atan2(dz, std::sqrt(dx * dx + dy * dy));
  return std::isfinite(angle) &&
         std::abs(angle - mount_angle_radians) <= threshold_radians;
}

}  // namespace lego_loam

#endif  // LEGO_LOAM_PROJECTION_MATH_H
