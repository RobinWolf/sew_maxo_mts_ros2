#ifndef DIFFDRIVE_WHEELS_TO_VEL_AND_DIR_H
#define DIFFDRIVE_WHEELS_TO_VEL_AND_DIR_H

#include <string>
#include <cmath>

class Wheel
{
public:
    std::string name = "";
    double cmd = 0;
    double pos = 0;
    double vel = 0;

    Wheel()
    {
        // Default constructor
    }

    void setName(const std::string &wheel_name)
    {
        name = wheel_name;
    }
};

class Wheels_to_vel_and_dir
{
public:
    Wheel left_wheel_;
    Wheel right_wheel_;
    double wheel_separation_;
    double wheel_radius_;

    Wheels_to_vel_and_dir()
    {
        // Default constructor
    }

    void set(const std::string &left_wheel_name, const std::string &right_wheel_name,
             const double &wheel_separation, const double &wheel_radius)
    {
        left_wheel_.setName(left_wheel_name);
        right_wheel_.setName(right_wheel_name);
        wheel_separation_ = wheel_separation;
        wheel_radius_ = wheel_radius;
    }
    // ###########################################################################################
    // ########### TODO: Implement the following function correct with correct returns ###########
    // ###########################################################################################

    void calculateVelocityAndDirection(double &linear_velocity, double &angular_velocity)
    {
        // Calculate linear and angular velocities
        linear_velocity = (right_wheel_.vel + left_wheel_.vel) * wheel_radius_ / 2.0;
        angular_velocity = (right_wheel_.vel - left_wheel_.vel) * wheel_radius_ / wheel_separation_;
    }
};

#endif // DIFFDRIVE_WHEELS_TO_VEL_AND_DIR_H
