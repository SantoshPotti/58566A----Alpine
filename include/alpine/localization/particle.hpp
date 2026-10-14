#pragma once

#include "alpine/pose.hpp"

// alpine/localization/particle.hpp
//
// LemLib groups a subsystem's types under a subfolder (chassis/chassis.hpp,
// chassis/odom.hpp). Alpine's subsystem is localization, so the filter, the
// field map, and the particle type live under include/alpine/localization/.

namespace alpine {

struct Particle {
    Pose pose;
    double weight = 0.0;

    Particle() = default;
    Particle(const Pose& p, double w) : pose(p), weight(w) {}
};

}  // namespace alpine