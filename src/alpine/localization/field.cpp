#include "alpine/localization/field.hpp"

#include <algorithm>
#include <cmath>

#include "alpine/util.hpp"

namespace alpine {

namespace {
// 2D cross product of (ax, ay) and (bx, by).
inline double cross(double ax, double ay, double bx, double by) {
    return ax * by - ay * bx;
}
constexpr double kParallelEps = 1e-12;
}  // namespace

Pose sensorPose(const Pose& robot, const SensorMount& mount) {
    const double c = std::cos(robot.theta);
    const double s = std::sin(robot.theta);
    return Pose(robot.x + mount.x * c - mount.y * s,
                robot.y + mount.x * s + mount.y * c,
                normalizeAngle(robot.theta + mount.theta));
}

Field Field::box(double width, double height) {
    const double hw = width / 2.0;
    const double hh = height / 2.0;
    Field f;
    f.addWall(Segment(-hw, -hh, hw, -hh));  // bottom
    f.addWall(Segment(hw, -hh, hw, hh));    // right
    f.addWall(Segment(hw, hh, -hw, hh));    // top
    f.addWall(Segment(-hw, hh, -hw, -hh));  // left
    return f;
}

void Field::addWall(const Segment& wall) {
    walls_.push_back(wall);
    const double loX = std::min(wall.x1, wall.x2);
    const double hiX = std::max(wall.x1, wall.x2);
    const double loY = std::min(wall.y1, wall.y2);
    const double hiY = std::max(wall.y1, wall.y2);
    if (!hasBounds_) {
        minX_ = loX; maxX_ = hiX; minY_ = loY; maxY_ = hiY;
        hasBounds_ = true;
    } else {
        minX_ = std::min(minX_, loX);
        maxX_ = std::max(maxX_, hiX);
        minY_ = std::min(minY_, loY);
        maxY_ = std::max(maxY_, hiY);
    }
}

double Field::raycast(const Pose& from, double maxRange) const {
    const double rx = std::cos(from.theta);
    const double ry = std::sin(from.theta);

    double best = maxRange;

    for (const Segment& w : walls_) {
        // Segment as A + u * s, ray as P + t * r.
        const double sx = w.x2 - w.x1;
        const double sy = w.y2 - w.y1;

        const double denom = cross(rx, ry, sx, sy);
        if (std::fabs(denom) < kParallelEps) continue;  // parallel

        const double qpx = w.x1 - from.x;
        const double qpy = w.y1 - from.y;

        const double t = cross(qpx, qpy, sx, sy) / denom;  // along the ray
        const double u = cross(qpx, qpy, rx, ry) / denom;  // along the segment

        if (t < 0.0) continue;             // wall is behind the sensor
        if (u < 0.0 || u > 1.0) continue;  // misses the finite segment
        if (t < best) best = t;
    }

    return best;
}

double Field::raycast(const Pose& robot, const SensorMount& mount) const {
    return raycast(sensorPose(robot, mount), mount.maxRange);
}

bool Field::contains(double x, double y, double margin) const {
    if (!hasBounds_) return false;
    return x >= minX_ + margin && x <= maxX_ - margin &&
           y >= minY_ + margin && y <= maxY_ - margin;
}

}  // namespace alpine