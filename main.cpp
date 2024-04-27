#define _USE_MATH_DEFINES
#define ERROR_BOUND 1e-5
#define DEBUG False

#include <iostream>
#include <matplot/matplot.h>
#include <set>
#include <cmath>
#include <stdexcept>

#include "config.h"
#include "src/controller.h"

// TODO
// Assumption: Light reaches every point in the field.
class Light {
public:
    std::array<double, 2> position;
    double radius;

public:
    Light(std::array<double, 2> position, double radius) {
        this->position = position;
        this->radius = radius;
    }

    double getIntensity(std::vector<double> sensor) {
        return std::sqrt(std::pow(sensor[0] - this->position[0], 2) + std::pow(sensor[1] - this->position[1], 2));
    }
};


// Convert an angle to a vector in two dimensional space.
//
// Args:
// - angle: Angle in degrees.
// - vectorLength: Length the vector should have.
//
// Returns:
//   Vector relative to the [0, 0] vector with the given length.
std::vector<double> angleToVector(double angle, double vectorLength = 1) {
    double x = std::cos(angle/180 * M_PI) * vectorLength/2;
    double y = std::sin(angle/180 * M_PI) * vectorLength/2;
    return std::vector<double>{x, y};
}


// TODO
bool isNearlyEqual(double a, double b) {
    return std::abs(a-b) < ERROR_BOUND;
}

class Robot {
public:
    // Where the robot is located (x, y).
    std::array<double, 2> position;
    // Angle where the robot is heading relative to its center.
    double heading;
    // Radius (just for visualization). The center of the circle represents the robot itself.
    double radius;
    // Angles of the light sensors relative to the angle of the heading;
    std::array<double, 2> sensors;

private:
    auto drawArrowRelativeToCenter(std::vector<double> vec) {
        return matplot::arrow(this->position[0], this->position[1], this->position[0] + vec[0], this->position[1] + vec[1]);
    }

public:
    // Constructor
    //
    // Args
    // - position: Initial position of the robot in a two dimensional space.
    // - heading: The way, the robot is heading based on its center/position (in degree).
    // - radius: Size of the robot.
    // - sensors: Angle where the sensors are located relative to the heading.
    Robot(std::array<double, 2> position, double heading, double radius, std::array<double, 2> sensors) {
        this->position = position;
        this->heading = heading;
        this->radius = radius;
        this->sensors = sensors;
    }

    // Drive the vehicle.
    // 
    // Args:
    // - speed: Speed of the wheels in unit/timeStep.
    // - timeSteps: Number of time steps to take. With increasing timeSteps the accuracy of the update is reduced.
    void drive(std::vector<double> speed, double timeSteps) {
        std::vector<double> newPosition;
        double rotationAngle;
        if (isNearlyEqual(speed[0], speed[1])) {
            // Straight forward.
            rotationAngle = 0;
            newPosition = angleToVector(this->heading + rotationAngle, speed[1] * timeSteps);
        } else if (isNearlyEqual(-speed[0], speed[1])) {
            // Rotate on spot.
            rotationAngle = (360 * speed[1] * timeSteps)/(2 * M_PI * this->radius);
            newPosition = {0, 0};
        } else {
            double rotationOffset = (this->radius) * (speed[1] + speed[0])/(speed[1] - speed[0]);
            rotationAngle = (360 * speed[1] * timeSteps)/(2 * M_PI * rotationOffset);
            const double headingAngleOffset = (180 - rotationAngle) / 2;
            const double headingLength = rotationOffset / std::tan(headingAngleOffset * M_PI/180);
            newPosition = angleToVector(this->heading + rotationAngle, headingLength);
        }

        this->position[0] += newPosition[0];
        this->position[1] += newPosition[1];

        this->heading += rotationAngle;
    }

    // TODO
    std::vector<double> getHeadingVector() {
        return angleToVector(this->heading, this->radius);
    }

    // TODO
    std::vector<double> getSensorVector(int idx) {
        if (idx > 1 || idx < 0) {
            throw std::invalid_argument("Index out of bounds");
        }

        return angleToVector(this->heading + this->sensors[idx], this->radius);
    }
};

// TODO
class Visualizer {
private:
    matplot::figure_handle f;
    matplot::axes_handle ax;

private:
    matplot::vectors_handle drawArrowRelativeToCenter(std::array<double, 2> center, std::vector<double> vec) {
        matplot::vectors_handle v = this->ax->arrow(center[0], center[1], center[0] + vec[0], center[1] + vec[1]);
        return v;
    }

public:
    Visualizer(const std::array<double, 2U> xlim, const std::array<double, 2U> ylim) {
        this->f = matplot::figure(true);
        this->ax = this->f->current_axes();
        this->ax->xlim(xlim);
        this->ax->ylim(ylim);
    }

    void drawRobot(Robot *rob) {
        this->ax->ellipse(rob->position[0]-(rob->radius / 2), rob->position[1]-(rob->radius / 2), rob->radius, rob->radius);
        matplot::vectors_handle v = this->drawArrowRelativeToCenter(rob->position, rob->getHeadingVector());
        v->color("red");
        this->drawArrowRelativeToCenter(rob->position, rob->getSensorVector(0));
        this->drawArrowRelativeToCenter(rob->position, rob->getSensorVector(1));
    }

    void drawLight(Light *light) {
        auto a = this->ax->ellipse(light->position[0]-(light->radius / 2), light->position[1]-(light->radius / 2), light->radius, light->radius);
        a->fill(true);
        a->color("green");
    }

    void show() {
        this->f->show();
    }

    void save(std::string filename) {
        this->f->save(filename);
    }
};

class Controller {
public:
    virtual std::vector<double> control(std::vector<double> sensorValues) = 0;
};

class BraitenbergAgressor : public Controller {
public:
     std::vector<double> control(std::vector<double> sensorValues) {
        return {1, 1};
     }
};

class BraitenbergFear : public Controller {
public:
     std::vector<double> control(std::vector<double> sensorValues) {
        return {1, 1};
     }
};

Controller* buildController(std::string identifier) {
    if (identifier == "agressor") {
        return new BraitenbergAgressor();
    } else if (identifier == "fear") {
        return new BraitenbergFear();
    } else {
        throw std::invalid_argument("Controller " + identifier + " unknown.");
    }
}

int main(int argc, char* argv[]) {
    std::cout << "You are running version " << EvoRob_VERSION_MAJOR << "." << EvoRob_VERSION_MINOR << "." << std::endl;

    const std::array<double, 2U> xlim = {-100, 100};
    const std::array<double, 2U> ylim = {-100, 100};

    Controller* controller;
    if (argc > 1) {
        controller = buildController(argv[1]);
    } else {
        controller = buildController("agressor");
    }


    try {
        Robot rob({93.5, 93.5}, 45.0, 5.0, {-45, 45});
        Light buzz({40, 40}, 10);
        Visualizer viz(xlim, ylim);

        viz.drawLight(&buzz);
        viz.drawRobot(&rob);
        for (int i = 0; i < 100; i++) {
            #ifdef DEBUG
                std::cout << "Iteration: " << i << " - Intensities: " << buzz.getIntensity(rob.getSensorVector(0)) << "," << buzz.getIntensity(rob.getSensorVector(1)) << std::endl;
            #endif

            rob.drive({1, 1}, 1);
            if (rob.position[0] > xlim[1]) {
                rob.position[0] -= (xlim[1] - xlim[0]);
            } else if (rob.position[0] < xlim[0]) {
                rob.position[0] += (xlim[1] - xlim[0]);
            }
            if (rob.position[1] > ylim[1]) {
                rob.position[1] -= (ylim[1] - ylim[0]);
            } else if (rob.position[0] < ylim[0]) {
                rob.position[1] += (ylim[1] - ylim[0]);
            }
            viz.drawRobot(&rob);
        }
        viz.save("path.svg");
        viz.show();
    } catch (const std::runtime_error &e) {
        // If you encounter the "popen() failed!" error, you likely have not
        // installed gnuplot correctly.
        std::cerr << "Runtime error: " << e.what() << std::endl;
    }
    return 0;
}
