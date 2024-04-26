#define _USE_MATH_DEFINES
#define ERROR_BOUND 1e-5
#define DEBUG False

#include <iostream>
#include <matplot/matplot.h>
#include <set>
#include <cmath>
#include <stdexcept>
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


inline bool isNearlyEqual(double x, double y)
{
  return std::abs(x - y) <= ERROR_BOUND;
}

// Convert an angle (in degrees) to a vector in two dimensional space.
std::vector<double> angleToVector(double angle, double vectorLength = 1) {
    double x = std::cos(angle/180 * M_PI) * vectorLength/2;
    double y = std::sin(angle/180 * M_PI) * vectorLength/2;
    if (isNearlyEqual(x, 0.0)) {
        y *= 2;
    }
    if (isNearlyEqual(y, 0.0)) {
        x *= 2;
    }
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
    // Angles of the light sensors relative to the angle of the heading;
    std::array<double, 2> sensors;
    // Speed of the left (idx 0) and right (idx 1) wheel.
    std::array<double, 2> speed;

private:
    auto drawArrowRelativeToCenter(std::vector<double> vec) {
        return matplot::arrow(this->position[0], this->position[1], this->position[0] + vec[0], this->position[1] + vec[1]);
    }

public:
    Robot(std::array<double, 2> position, double heading, double radius, std::array<double, 2> speed, std::array<double, 2> sensors) {
        this->position = position;
        this->heading = heading;
        this->radius = radius;
        this->speed = speed;
        this->sensors = sensors;
    }

    void drive(std::vector<double> speed, double timeSteps) {
        // Speed in unit/timeStep.
        const double rotation_offset = (this->radius) * (speed[1] + speed[0])/(speed[1] - speed[0]);
        const double rotation_angle = (360 * speed[1] * timeSteps)/(2 * M_PI * rotation_offset);
        #ifdef DEBUG
            std::vector<double> rotation_point = angleToVector(this->heading + 90, rotation_offset);
            this->drawArrowRelativeToCenter(rotation_point);
        #endif

        const double heading_angle_offset = (180 - rotation_angle) / 2;
        const double heading_length = rotation_offset / std::tan(heading_angle_offset * M_PI/180);
        const std::vector<double> new_position = angleToVector(this->heading + rotation_angle, heading_length);
        this->position[0] += new_position[0];
        this->position[1] += new_position[1];
        this->heading += rotation_angle;

        #ifdef DEBUG
            std::cout << rotation_angle << " " << rotation_offset  << " " << heading_length << " " << heading_angle_offset << " " << speed[0] * timeSteps << std::endl;
        #endif
        // double dl = speed[0] * timeSteps;
        // double dr = speed[1] * timeSteps;

        
        
        // double x = rotation_offset * std::cos(dr / (this->radius + rotation_offset)) - rotation_offset;
        // double y = rotation_offset * std::sin(dr / (this->radius + rotation_offset));
        // std::cout << new_angle << " " << rotation_offset << " " << x << " " << y << std::endl;



        // this->heading = this->heading - new_angle;

        // this->position[0] = this->position[0] + x - (this->radius / 2);
        // this->position[1] = this->position[1] + y -(this->radius / 2);
        
        // std::vector<double> vec = angleToVector(this->heading + 90, s);
        // std::cout << "S: " << s << " x" << vec[0] << vec[1] << std::endl;
        // this->drawArrowRelativeToCenter(vec);


        // double x = s * std::cos(dr / (this->radius + s)) - s;
        // double y = s * std::sin(dr / (this->radius + s));

        // // this->position

        // matplot::ellipse(this->position[0] + x - (this->radius / 2), this->position[1] + y -(this->radius / 2), this->radius, this->radius);




        // double angular_velocity = (speed[1]-speed[0])/(this->radius*2);
        // double translation = ((speed[1]+speed[0])/2) * timeSteps;
        // std::cout << "-----" << std::endl;
        // std::cout << "Angular velocity: " << angular_velocity << std::endl;
        // std::cout << "Heading before: " << this->heading << std::endl;
        // std::cout << "Current position: " << this->position[0] << "," << this->position[1] << std::endl;
        // this->heading += angular_velocity * timeSteps;
        // std::cout << std::cos(this->heading) << std::endl;
        // this->position[0] += translation * std::cos(this->heading * M_PI/180);
        // this->position[1] += translation * std::sin(this->heading * M_PI/180);
        // std::cout << "Next position: " << this->position[0] << "," << this->position[1] << std::endl;
        // std::cout << "Heading after: " << this->heading << std::endl;
        
    }

    std::vector<double> getHeadingVector() {
        return angleToVector(this->heading, this->radius);
    }

    std::vector<double> getSensorVector(int idx) {
        if (idx > 1 || idx < 0) {
            throw std::invalid_argument("Index out of bounds");
        }

        return angleToVector(this->heading + this->sensors[idx]);
    }

    void draw() {
        std::vector<double> vec = this->getHeadingVector();
        matplot::ellipse(this->position[0]-(this->radius / 2), this->position[1]-(this->radius / 2), this->radius, this->radius);
        // Heading direction.
        auto a = this->drawArrowRelativeToCenter(this->getHeadingVector());
        a->color("red");
        // Sensors.
        this->drawArrowRelativeToCenter(this->getSensorVector(0));
        this->drawArrowRelativeToCenter(this->getSensorVector(1));
    }
};



int main()
{
    std::cout << "You are running version " << EvoRob_VERSION_MAJOR << "." << EvoRob_VERSION_MINOR << "." << std::endl;
    try
    {
        Robot rob({0.1, 0.1}, 90, 0.5, {100.0, 100.0}, {-45, 45});
        Light buzz({0.75, 0.75}, 0.1);
        rob.draw();
        for (int i = 0; i <= 25; i++) {
            rob.drive({0.075, 0.1}, 1);
            rob.draw();
        }
        buzz.draw();

        std::vector<double> vec = {buzz.position[0] - rob.position[0], buzz.position[1] - rob.position[1]};
        std::cout << buzz.position[0] << " - " << buzz.position[1] << std::endl;
        std::cout << rob.position[0] << " - " << rob.position[1] << std::endl;
        std::cout << vec[0] << " - " << vec[1] << std::endl;

        // Sensor: needs an angle which is relative to the heading of the robot.
        // Get distance between the sensor and the light
        // 1. Create
        
        matplot::xlim({-2, 2});
        matplot::ylim({-2, 2});
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
