#pragma once

// alpine/pose.hpp
//
// Matches the lemlib/pose.hpp placement: the one geometric type that shows up
// in every other header lives at the top level, not buried in a subfolder.
//
// This header has no PROS dependency, which is what lets the whole filter be
// compiled and tested with plain g++ on a laptop. Only the Phase 6 adapter
// under include/alpine/pros/ is allowed to include <pros/...>.
//
// CONVENTIONS (fixed here once, obeyed everywhere):
//   World frame : origin at field center, +x right, +y forward (away from the
//                 driver station), theta in RADIANS counterclockwise from +x.
//                 Distances in inches.
//   Robot frame : +x forward, +y left. A robot at theta = 0 faces world +x.

namespace alpine {

struct Pose {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    Pose() = default;
    Pose(double x_, double y_, double theta_ = 0.0)
        : x(x_), y(y_), theta(theta_) {}
};

}  // namespace alpine