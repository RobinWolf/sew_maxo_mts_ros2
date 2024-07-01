#ifndef DIFFDRIVE_WHEELS_TO_VEL_AND_DIR_H
#define DIFFDRIVE_WHEELS_TO_VEL_AND_DIR_H

#include <string>
#include <iostream>
#include <cmath>

struct Wheel
{
    std::string name = "";
    double cmd = 0;
    double pos = 0;
    double vel = 0;
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

    // void set(const std::string &left_wheel_name, const std::string &right_wheel_name,
    //          const double &wheel_separation, const double &wheel_radius)
    // {
    //     left_wheel_.setName(left_wheel_name);
    //     right_wheel_.setName(right_wheel_name);
    //     wheel_separation_ = wheel_separation;
    //     wheel_radius_ = wheel_radius;
    // }
    

    void getVelocityAndDirection(float &speed, float &x, float &y)
    {

        // ###########################################################################################
        // TODO: Implement the following function correct with correct returns
        // command vel for both wheels gets stored in left_wheel_.cmd and right_wheel_.cmd --> use this to calculate the speed, x and y (comes from the controller)
        // state vel for both wheels gets stored in left_wheel_.vel and right_wheel_.vel --> needs to be calculated here because no info from agv?
        // state pos for both wheels gets stored in left_wheel_.pos and right_wheel_.pos --> needs to be calculated here because no info from agv?
        // ###########################################################################################

        // dummy values for testing --> drive straight forward
        // x = 1.0;            // right
        // y = 0.0;           // forward
        // speed = 100.0;       // speed between 0 and 100

        // Radgeschwindigkeit berechnen (v_left und v_right)
        double v_left = left_wheel_.cmd * wheel_radius_;
        double v_right = right_wheel_.cmd * wheel_radius_;

        // Vorwärtsgeschwindigkeit (v) und Drehgeschwindigkeit (omega) berechnen
        double v = (v_left + v_right) / 2.0;
        double omega = (v_right - v_left) / wheel_separation_;

        // Da wir nur die x- und y-Komponenten der Geschwindigkeit berechnen wollen,
        // nehmen wir an, dass der Roboter in Richtung seiner Vorwärtsachse bewegt wird.
        // Daher sind die Geschwindigkeiten in x- und y-Richtungen:
        y = v * cos(omega)*10; // Geschwindigkeit nach vorne
        x = v * sin(omega)*10; // Geschwindigkeit nach rechts

        // Setze die Geschwindigkeit als Betrag der Vorwärtsgeschwindigkeit
        // speed = std::sqrt(x * x + y * y);
        speed = 100;

        std::cout << "Übergebene Werte:" << std::endl;
        std::cout << "left cmd: " << left_wheel_.cmd << std::endl;
        std::cout << "right cmd: " << right_wheel_.cmd << std::endl;

        // Ausgabe zur Überprüfung der Werte
        std::cout << "Berechnete Richtung und Geschwindigkeit:" << std::endl;
        std::cout << "x: " << x << std::endl;
        std::cout << "y: " << y << std::endl;
        std::cout << "speed: " << speed << std::endl;
    }
};

#endif // DIFFDRIVE_WHEELS_TO_VEL_AND_DIR_H
