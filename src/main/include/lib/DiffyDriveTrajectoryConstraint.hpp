#pragma once

#include <lib/DiffyDriveModel.hpp>
#include <frc/trajectory/constraint/TrajectoryConstraint.h>
#include <units/pressure.h>

namespace rj
{

    class DiffyDriveTrajectoryConstraint : public frc::TrajectoryConstraint
    {
    private:
        units::meters_per_second_t clamp(units::meters_per_second_t value, units::meters_per_second_t bound)
        {
            return units::math::min(units::math::max(value, bound), -bound);
        }

    public:
        const DiffyDriveModel drivetrain_model;
        const units::pounds_per_square_inch_t vacuum;

        DiffyDriveTrajectoryConstraint(const DiffyDriveModel model, units::pounds_per_square_inch_t vacuum = 1_psi) : drivetrain_model(model), vacuum(vacuum) {}
        /**
     * Returns the max velocity given the current pose and curvature.
     *
     * @param pose The pose at the current point in the trajectory.
     * @param curvature The curvature at the current point in the trajectory.
     * @param velocity The velocity at the current point in the trajectory before
     *                                constraints are applied.
     *
     * @return The absolute maximum velocity.
     */
        units::meters_per_second_t
        MaxVelocity(const frc::Pose2d &pose, units::curvature_t curvature, units::meters_per_second_t velocity) const
        {
            units::meters_per_second_t velocity_left = drivetrain_model.MaxVelocity() * (2 - drivetrain_model.track_width * curvature / 1_rad) / 2;
            units::meters_per_second_t velocity_right = drivetrain_model.MaxVelocity() * (2 + drivetrain_model.track_width * curvature / 1_rad) / 2;

            auto actual_max = units::math::max(units::math::abs(velocity_left), units::math::abs(velocity_right));

            velocity_left *= drivetrain_model.MaxVelocity() / actual_max;
            velocity_right *= drivetrain_model.MaxVelocity() / actual_max;

            velocity = (velocity_left + velocity_right) / 2;

            units::force::newton_t available_downforce = (drivetrain_model.mass * 9.80665_mps_sq + (vacuum * 130_sq_in));
            units::force::newton_t forward_force = drivetrain_model.State(velocity).force_on_surface;
            units::force::newton_t centripetal_force = drivetrain_model.mass * velocity * velocity * curvature / 1_rad;
            while (available_downforce * available_downforce < forward_force * forward_force + centripetal_force * centripetal_force)
            {
                velocity -= 0.1_mps;
                forward_force = drivetrain_model.State(velocity).force_on_surface;
                centripetal_force = drivetrain_model.mass * velocity * velocity * curvature / 1_rad;
            }

            velocity += 0.1_mps;
            forward_force = drivetrain_model.State(velocity).force_on_surface;
            centripetal_force = drivetrain_model.mass * velocity * velocity * curvature / 1_rad;
            while (available_downforce * available_downforce < forward_force * forward_force + centripetal_force * centripetal_force)
            {
                velocity -= 0.01_mps;
                forward_force = drivetrain_model.State(velocity).force_on_surface;
                centripetal_force = drivetrain_model.mass * velocity * velocity * curvature / 1_rad;
            }

            return velocity;
        }

        /**
     * Returns the minimum and maximum allowable acceleration for the trajectory
     * given pose, curvature, and speed.
     *
     * @param pose The pose at the current point in the trajectory.
     * @param curvature The curvature at the current point in the trajectory.
     * @param speed The speed at the current point in the trajectory.
     *
     * @return The min and max acceleration bounds.
     */
        frc::TrajectoryConstraint::MinMax MinMaxAcceleration(const frc::Pose2d &pose, units::curvature_t curvature, units::meters_per_second_t speed) const
        {
            /* There are problems with this approach - namely that at high curvature and low velocity 
            your acceleration can be forced negative
           and consequently in a path with high initial curvature this fails to calculate positive initial acceleration
           although clearly we can achieve the desired velocity / acceleration pair
           The end goal for this approach is to get an overall acceleration that is constrained by the possible acceleration on either side
           So, for now, just get the overall potential acceleration
        auto velocity_left = clamp(speed * (2 - drivetrain_model.track_width * curvature) / 2, drivetrain_model.MaxVelocity());
        auto velocity_right = clamp(speed * (2 + drivetrain_model.track_width * curvature) / 2, drivetrain_model.MaxVelocity());
        auto accel_left = drivetrain_model.State(velocity_left).wheel_acceleration;
        auto accel_right = drivetrain_model.State(velocity_right).wheel_acceleration;
        auto accel_from_left = 2.0 * accel_left / (2.0 - drivetrain_model.track_width * curvature);
        auto accel_from_right = 2.0 * accel_right / (2.0 + drivetrain_model.track_width * curvature);
        */
            auto middle_accel = drivetrain_model.State(speed).wheel_acceleration;

            return {
                minAcceleration : -drivetrain_model.MaxAccel(),
                maxAcceleration : middle_accel
            };
        }
    };

} // namespace rj