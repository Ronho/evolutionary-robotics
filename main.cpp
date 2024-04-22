#include <iostream>
#include <matplot/matplot.h>
#include <set>
#include <cmath>
#include "config.h"

class Light {
public:
    std::array<double, 2> position;
    double radius;

public:
    Light(std::array<double, 2> position, double radius) {
        this->position = position;
        this->radius = radius;
    }

    void draw() {
        auto a = matplot::ellipse(this->position[0]-(this->radius / 2), this->position[1]-(this->radius / 2), this->radius, this->radius);
        a->fill(true);
        a->color("green");
    }
};

// Convert an angle (in degrees) to a vector in two dimensional space.
std::vector<double> angleToVector(double angle, double vectorLength = 1) {
    double x = std::cos(angle * M_PI/180) * vectorLength/2;
    double y = std::sin(angle * M_PI/180) * vectorLength/2;

    return std::vector<double>{x, y};
}

class Robot {
public:        // Access specifier
    // Where the robot is located (x, y).
    std::array<double, 2> position;
    // Angle where the robot is heading relative to its center.
    double heading;
    // Radius (just for visualization). The center of the circle represents the robot itself.
    double radius;
    // Angles of the sensors relative to the angle of the heading;
    std::array<double, 2> sensors;
    // Speed of the left (idx 0) and right (idx 1) wheel.
    std::array<double, 2> speed;

public:
    Robot(std::array<double, 2> position, double heading, double radius, std::array<double, 2> speed, std::array<double, 2> sensors) {
        this->position = position;
        this->heading = heading;
        this->radius = radius;
        this->speed = speed;
        this->sensors = sensors;
    }

    std::vector<double> getHeadingVector() {
        double x = std::cos(heading * M_PI/180) * radius/2;
        double y = std::sin(heading * M_PI/180) * radius/2;

        return std::vector<double>{x, y};
    }

    void draw() {
        std::vector<double> vec = angleToVector(this->heading, this->radius);
        matplot::ellipse(this->position[0]-(this->radius / 2), this->position[1]-(this->radius / 2), this->radius, this->radius);
        // Heading direction.
        auto a = matplot::arrow(this->position[0], this->position[1], this->position[0] + vec[0], this->position[1] + vec[1]);
        a->color("red");
        // Sensors.
        vec = angleToVector(this->heading + this->sensors[0]);
        matplot::arrow(this->position[0], this->position[1], this->position[0] + vec[0], this->position[1] + vec[1]);
        vec = angleToVector(this->heading + this->sensors[1]);
        matplot::arrow(this->position[0], this->position[1], this->position[0] + vec[0], this->position[1] + vec[1]);
    }
};



int main()
{
    std::cout << "You are running version " << EvoRob_VERSION_MAJOR << "." << EvoRob_VERSION_MINOR << "." << std::endl;
    try
    {
        Robot rob({0.5, 0.5}, 90, 0.5, {100.0, 100.0}, {-45, 45});
        Light buzz({0.75, 0.75}, 0.1);
        rob.draw();
        buzz.draw();

        // Sensor: needs an angle which is relative to the heading of the robot.
        // Get distance between the sensor and the light
        // 1. Create
        
        matplot::xlim({0, 1});
        matplot::ylim({0, 1});
        matplot::show();
    }
    catch (const std::runtime_error &e)
    {
        // If you encounter the "popen() failed!" error, you likely have not
        // installed gnuplot correctly.
        std::cerr << "Runtime error: " << e.what() << std::endl;
    }
    return 0;
}
