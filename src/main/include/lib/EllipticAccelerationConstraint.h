#pragma once

#include <frc/trajectory/constraint/TrajectoryConstraint.h>
#include <units/acceleration.h>
#include <units/curvature.h>
#include <units/velocity.h>

namespace frc
{

class EllipticAccelerationConstraint : public TrajectoryConstraint
{
public:
    explicit EllipticAccelerationConstraint(
        units::meters_per_second_squared_t maxCentripetalAcceleration,
        units::meters_per_second_squared_t maxLinearAcceleration) : 
        m_maxCentripetalAcceleration{maxCentripetalAcceleration},
        m_maxLinearAcceleration{maxLinearAcceleration} {};

    units::meters_per_second_t MaxVelocity(
        const Pose2d &pose, units::curvature_t curvature,
        units::meters_per_second_t velocity) const override
    {
        // curvature guides our max speed through turns
        return units::math::sqrt(m_maxCentripetalAcceleration /
                                    units::math::abs(curvature) * 1_rad);
    };

    MinMax MinMaxAcceleration(const Pose2d &pose, units::curvature_t curvature,
                                units::meters_per_second_t speed) const override
    {
        // Actual centripetal acceleration
        auto max_centripetal_accel_at_point = speed * speed * curvature / 1_rad;

        auto max_linear_accel = units::math::sqrt(
            m_maxLinearAcceleration * m_maxLinearAcceleration 
            + m_maxCentripetalAcceleration * m_maxCentripetalAcceleration 
            - max_centripetal_accel_at_point * max_centripetal_accel_at_point);

        return {-max_linear_accel, max_linear_accel};
    };

private:
    units::meters_per_second_squared_t m_maxCentripetalAcceleration;
    units::meters_per_second_squared_t m_maxLinearAcceleration;
};

} // namespace frc