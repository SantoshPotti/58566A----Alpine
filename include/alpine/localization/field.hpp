#pragma once

#include <vector>

#include "alpine/pose.hpp"

// alpine/localization/field.hpp
//
// The map the particle filter measures against. A Field is a list of line
// segments, so it covers the four perimeter walls and any static barrier you
// choose to model. It answers one question: standing here, looking that way,
// how far to the nearest wall?

namespace alpine {

struct Segment {
    double x1 = 0.0, y1 = 0.0;
    double x2 = 0.0, y2 = 0.0;

    Segment() = default;
    Segment(double ax, double ay, double bx, double by)
        : x1(ax), y1(ay), x2(bx), y2(by) {}
};

// Where a distance sensor sits on the robot, in the ROBOT frame
// (+x forward, +y left), and which way it faces relative to robot heading.
struct SensorMount {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    // Max trustworthy reading, inches. The V5 Distance Sensor tops out around
    // 78 in, so on a 144 in field it frequently cannot see the far wall.
    // Phase 3 must treat an out-of-range reading as evidence, not as an error.
    double maxRange = 78.0;

    SensorMount() = default;
    SensorMount(double x_, double y_, double theta_, double maxRange_ = 78.0)
        : x(x_), y(y_), theta(theta_), maxRange(maxRange_) {}
};

// Lifts a mount into world coordinates given the robot's pose.
Pose sensorPose(const Pose& robot, const SensorMount& mount);

class Field {
public:
    Field() = default;

    // Axis-aligned box centered on the origin. On a standard V5 field the
    // inside-wall span is about 140.5 in, not the nominal 144 in. Measure
    // yours and pass the real number.
    static Field box(double width, double height);

    void addWall(const Segment& wall);
    const std::vector<Segment>& walls() const { return walls_; }

    // Distance from `from` along its own heading to the nearest wall.
    // Returns maxRange when nothing is hit inside maxRange.
    double raycast(const Pose& from, double maxRange = 1e9) const;

    // Convenience: raycast from a sensor bolted to a robot at `robot`.
    double raycast(const Pose& robot, const SensorMount& mount) const;

    // True if the point lies inside the bounding box of every wall added so
    // far. Used in Phase 4 to kill particles that drifted through a wall.
    bool contains(double x, double y, double margin = 0.0) const;

private:
    std::vector<Segment> walls_;
    double minX_ = 0.0, maxX_ = 0.0, minY_ = 0.0, maxY_ = 0.0;
    bool hasBounds_ = false;
};

}  // namespace alpine