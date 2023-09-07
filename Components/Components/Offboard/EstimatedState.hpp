/* An estimator that uses measurements from the GPS, and generates
 * an estimate of the vehicle's position, velocity, orientation, and angular velocity.
 */

#pragma once

#include <Eigen/Dense>
#include "Common/Math/Vec3.hpp"
#include "Common/Math/Rotation.hpp"

namespace Offboard {
    struct EstimatedState {
        Vec3d pos, vel, angVel;
        Rotationd att;
        };
}