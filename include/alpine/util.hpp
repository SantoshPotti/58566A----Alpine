#pragma once

#include <cmath>

// alpine/util.hpp
//
// Small free functions with no dependencies, split out the same way
// lemlib/util.hpp is.

namespace alpine {

// Wraps an angle into (-pi, pi].
inline double normalizeAngle(double radians) {
    double a = std::fmod(radians + M_PI, 2.0 * M_PI);
    if (a <= 0.0) a += 2.0 * M_PI;
    return a - M_PI;
}

inline double degToRad(double degrees) { return degrees * M_PI / 180.0; }
inline double radToDeg(double radians) { return radians * 180.0 / M_PI; }

// Converts a VEX inertial heading (clockwise degrees from +y) into Alpine's
// world theta (counterclockwise radians from +x). Used by the Phase 6 adapter.
inline double headingToTheta(double imuDegrees) {
    return normalizeAngle(degToRad(90.0 - imuDegrees));
}

// The inverse, for reporting a pose back in terms the drive code expects.
inline double thetaToHeading(double theta) {
    double deg = std::fmod(90.0 - radToDeg(theta), 360.0);
    if (deg < 0.0) deg += 360.0;
    return deg;
}

}  // namespace alpine