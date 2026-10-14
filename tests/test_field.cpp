// tests/test_field.cpp
//
// Phase 1 checks. Every expected value here was worked out by hand, not by
// running the code and writing down whatever came out. That distinction is the
// whole point of the test.
//
// LemLib uses GoogleTest here. This is a dependency-free harness instead, so
// `make -C tests` works on a fresh laptop with nothing but g++. Swapping to
// GoogleTest later is a contained change to this file and the test Makefile.

#include <cmath>
#include <cstdio>
#include <string>

#include "alpine/api.hpp"

namespace {

int gFailures = 0;
int gChecks = 0;

void expectNear(const std::string& name, double got, double want,
                double tol = 1e-9) {
    ++gChecks;
    if (std::fabs(got - want) > tol) {
        std::printf("  FAIL  %-42s got %.6f, want %.6f\n", name.c_str(), got,
                    want);
        ++gFailures;
    } else {
        std::printf("  ok    %-42s %.6f\n", name.c_str(), got);
    }
}

void expectTrue(const std::string& name, bool cond) {
    ++gChecks;
    if (!cond) {
        std::printf("  FAIL  %-42s expected true\n", name.c_str());
        ++gFailures;
    } else {
        std::printf("  ok    %-42s\n", name.c_str());
    }
}

}  // namespace

int main() {
    using namespace alpine;

    // A 144 x 144 box centered on the origin puts walls at x = +/-72, y = +/-72.
    const Field field = Field::box(144.0, 144.0);

    std::printf("raycast, robot at field center\n");
    expectNear("facing +x", field.raycast(Pose(0, 0, 0)), 72.0);
    expectNear("facing +y", field.raycast(Pose(0, 0, M_PI / 2)), 72.0);
    expectNear("facing -x", field.raycast(Pose(0, 0, M_PI)), 72.0);
    expectNear("facing -y", field.raycast(Pose(0, 0, -M_PI / 2)), 72.0);
    // At 45 degrees the ray reaches the corner: 72 / cos(45) = 72 * sqrt(2).
    expectNear("facing 45 deg, hits corner",
               field.raycast(Pose(0, 0, M_PI / 4)), 72.0 * std::sqrt(2.0));

    std::printf("\nraycast, robot off center\n");
    expectNear("at x=36 facing +x", field.raycast(Pose(36, 0, 0)), 36.0);
    expectNear("at y=-36 facing +y", field.raycast(Pose(0, -36, M_PI / 2)),
               108.0);
    expectNear("at (60,60) facing +x", field.raycast(Pose(60, 60, 0)), 12.0);

    std::printf("\nmaxRange clamping\n");
    expectNear("wall at 72, maxRange 50", field.raycast(Pose(0, 0, 0), 50.0),
               50.0);

    std::printf("\nsensorPose transform (robot frame: +x fwd, +y left)\n");
    Pose p = sensorPose(Pose(10, 20, 0), SensorMount(5, 0, 0));
    expectNear("fwd offset, heading 0, x", p.x, 15.0);
    expectNear("fwd offset, heading 0, y", p.y, 20.0);

    p = sensorPose(Pose(10, 20, 0), SensorMount(0, 3, 0));
    expectNear("left offset, heading 0, x", p.x, 10.0);
    expectNear("left offset, heading 0, y", p.y, 23.0);

    p = sensorPose(Pose(10, 20, M_PI / 2), SensorMount(5, 0, 0));
    expectNear("fwd offset, heading 90, x", p.x, 10.0);
    expectNear("fwd offset, heading 90, y", p.y, 25.0);
    expectNear("fwd offset, heading 90, theta", p.theta, M_PI / 2);

    p = sensorPose(Pose(0, 0, 0), SensorMount(0, 0, M_PI / 2));
    expectNear("rotated mount inherits angle", p.theta, M_PI / 2);

    std::printf("\nraycast through a mount\n");
    expectNear("6 in forward mount sees 72-6",
               field.raycast(Pose(0, 0, 0), SensorMount(6, 0, 0)), 66.0);
    expectNear("left-facing mount at center",
               field.raycast(Pose(0, 0, 0), SensorMount(0, 0, M_PI / 2)), 72.0);
    // Far wall is 142 in away, past the sensor's 78 in ceiling.
    expectNear("out of range returns maxRange",
               field.raycast(Pose(-70, 0, 0), SensorMount(0, 0, 0)), 78.0);

    std::printf("\ncontains\n");
    expectTrue("center is inside", field.contains(0, 0));
    expectTrue("outside the wall is not inside", !field.contains(75, 0));
    expectTrue("margin rejects hugging the wall",
               !field.contains(70, 0, /*margin=*/6.0));

    std::printf("\nangle utilities\n");
    expectNear("3pi wraps to pi", normalizeAngle(3 * M_PI), M_PI);
    expectNear("-3pi/2 wraps to pi/2", normalizeAngle(-3 * M_PI / 2), M_PI / 2);
    expectNear("0 stays 0", normalizeAngle(0.0), 0.0);

    std::printf("\nIMU heading conversion (VEX cw-from-+y <-> world ccw-from-+x)\n");
    expectNear("imu 0 deg is world +y", headingToTheta(0.0), M_PI / 2);
    expectNear("imu 90 deg is world +x", headingToTheta(90.0), 0.0);
    expectNear("imu 180 deg is world -y", headingToTheta(180.0), -M_PI / 2);
    expectNear("round trip 0", thetaToHeading(headingToTheta(0.0)), 0.0, 1e-9);
    expectNear("round trip 137", thetaToHeading(headingToTheta(137.0)), 137.0,
               1e-9);

    std::printf("\n%d checks, %d failures\n", gChecks, gFailures);
    return gFailures == 0 ? 0 : 1;
}