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
    

    void getVelocityAndDirection(float &speed, float &x, float &y)
    {

        // ###########################################################################################
        // TODO: Implement the following function correct with correct returns
        // command vel for both wheels gets stored in left_wheel_.cmd and right_wheel_.cmd --> use this to calculate the speed, x and y (comes from the controller)
        // state vel for both wheels gets stored in left_wheel_.vel and right_wheel_.vel --> needs to be calculated here because no info from agv?
        // state pos for both wheels gets stored in left_wheel_.pos and right_wheel_.pos --> needs to be calculated here because no info from agv?
        // ###########################################################################################

        // dummy values for testing --> drive straight forward
        x = 1.0;
        y = 0.0;
        speed = 1.0;


    }
};

#endif // DIFFDRIVE_WHEELS_TO_VEL_AND_DIR_H
