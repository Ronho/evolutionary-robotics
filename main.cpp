#define _USE_MATH_DEFINES
#define DEBUG False

#include <iostream>
#include <matplot/matplot.h>
#include <set>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "config.h"
#include "utils.h"
#include "controller.h"
#include "types.h"

// TODO
// Assumption: Light reaches every point in the field.
class Light {
public:
    fixedVector position;
    double radius;

public:
    Light(fixedVector position, double radius) {
        this->position = position;
        this->radius = radius;
    }

    double getIntensity(fixedVector sensor, double norm) {
        return 1 - (calcLength(sensor[0], sensor[1], this->position[0], this->position[1]) / norm);
    }
};

class Robot {
public:
    // Where the robot is located (x, y).
    fixedVector position;
    // Angle where the robot is heading relative to its center.
    double heading;
    // Radius (just for visualization). The center of the circle represents the robot itself.
    double radius;
    // Angles of the light sensors relative to the angle of the heading;
    fixedVector sensors;

public:
    // Constructor
    //
    // Args
    // - position: Initial position of the robot in a two dimensional space.
    // - heading: The way, the robot is heading based on its center/position (in degree).
    // - radius: Size of the robot.
    // - sensors: Angle where the sensors are located relative to the heading.
    Robot(fixedVector position, double heading, double radius, fixedVector sensors) {
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
    void drive(fixedVector speed, double timeSteps) {
        fixedVector newPosition;
        double rotationAngle;
        if (isNearlyEqual(speed[0], speed[1])) {
            // Straight forward.
            rotationAngle = 0;
            newPosition = angleToVector(this->heading + rotationAngle, speed[1] * timeSteps);
        } else if (isNearlyEqual(-speed[0], speed[1])) {
            // Rotate on spot.
            rotationAngle = (360 * speed[0] * timeSteps)/(2 * M_PI * this->radius);
            newPosition = {0, 0};
        } else {
            double rotationOffset = (this->radius) * (speed[0] + speed[1])/(speed[0] - speed[1]);
            rotationAngle = (360 * speed[0] * timeSteps)/(2 * M_PI * rotationOffset);
            const double headingAngleOffset = (180 - rotationAngle) / 2;
            const double headingLength = rotationOffset / std::tan(headingAngleOffset * M_PI/180);
            newPosition = angleToVector(this->heading + rotationAngle, headingLength);
        }

        this->position[0] += newPosition[0];
        this->position[1] += newPosition[1];

        this->heading += rotationAngle;
    }

    // TODO
    fixedVector getHeadingVector() {
        return angleToVector(this->heading, this->radius);
    }

    // TODO
    fixedVector getRelativeSensorVector(int idx) {
        if (idx > 1 || idx < 0) {
            throw std::invalid_argument("Index out of bounds");
        }

        return angleToVector(this->heading + this->sensors[idx], this->radius);
    }

    fixedVector getSensorPosition(int idx) {
        fixedVector relativeVec = this->getRelativeSensorVector(idx);
        return {this->position[0] + relativeVec[0], this->position[1] + relativeVec[1]};
    }
};

// TODO
class Visualizer {
private:
    matplot::figure_handle f;
    matplot::axes_handle ax;

private:
    matplot::vectors_handle drawArrowRelativeToCenter(fixedVector center, fixedVector vec) {
        matplot::vectors_handle v = this->ax->arrow(center[0], center[1], center[0] + vec[0], center[1] + vec[1]);
        return v;
    }

public:
    Visualizer(const fixedVector xlim, const fixedVector ylim) {
        this->f = matplot::figure(true);
        this->ax = this->f->current_axes();
        this->ax->xlim(xlim);
        this->ax->ylim(ylim);
    }

    void drawRobot(Robot *rob) {
        // this->ax->ellipse(rob->position[0]-rob->radius, rob->position[1]-rob->radius, rob->radius * 2, rob->radius * 2);
        matplot::vectors_handle v = this->drawArrowRelativeToCenter(rob->position, rob->getHeadingVector());
        v->color("red");
        // this->drawArrowRelativeToCenter(rob->position, rob->getRelativeSensorVector(0));
        // this->drawArrowRelativeToCenter(rob->position, rob->getRelativeSensorVector(1));
    }

    void drawLight(Light *light) {
        auto a = this->ax->ellipse(light->position[0]-light->radius, light->position[1]-light->radius, light->radius * 2, light->radius * 2);
        a->fill(true);
        a->color("green");
    }

    void show() {
        this->f->show();
    }

    void update() {
        this->f->draw();
    }

    void save(std::string filename) {
        this->f->save(filename);
    }
};

class Torus {
public:
    fixedVector xlim;
    fixedVector ylim;

public:
    Torus(fixedVector xlim, fixedVector ylim) {
        this->xlim = xlim;
        this->ylim = ylim;
    }

    fixedVector clip(fixedVector position) {
        if (position[0] > this->xlim[1]) {
            position[0] -= (this->xlim[1] - this->xlim[0]);
        } else if (position[0] < xlim[0]) {
            position[0] += (this->xlim[1] - this->xlim[0]);
        }
        if (position[1] > this->ylim[1]) {
            position[1] -= (this->ylim[1] - this->ylim[0]);
        } else if (position[1] < ylim[0]) {
            position[1] += (this->ylim[1] - this->ylim[0]);
        }
        return position;
    }
};

int main(int argc, char* argv[]) {
    std::cout << "You are running version " << EvoRob_VERSION_MAJOR << "." << EvoRob_VERSION_MINOR << "." << std::endl;

    Torus env({-100, 100}, {-100, 100});
    double norm = calcLength(env.xlim[0], env.ylim[0], env.xlim[1], env.ylim[1]);

    Controller* controller;
    if (argc > 1) {
        controller = buildController(argv[1]);
    } else {
        // Fallback controller.
        controller = buildController("fear");
    }


    try {
        Robot rob({-90, -90}, 90, 5.0, {-45, 45});
        Light buzz({40, 40}, 10);
        Visualizer viz(env.xlim, env.ylim);

        viz.drawLight(&buzz);
        viz.drawRobot(&rob);
        viz.update();
        for (int i = 0; i < 1000; i++) {
            #ifdef DEBUG
                std::cout << "Iteration: " << i << " - Position: " << rob.position[0] << "," << rob.position[1] << std::endl;
                std::cout << "Iteration: " << i << " - Intensities: " << buzz.getIntensity(rob.getSensorPosition(0), norm) << "," << buzz.getIntensity(rob.getSensorPosition(1), norm) << std::endl;
            #endif

            fixedVector sensedValues = {buzz.getIntensity(rob.getSensorPosition(0), norm), buzz.getIntensity(rob.getSensorPosition(1), norm)};
            rob.drive(controller->control(sensedValues), 1);
            rob.position = env.clip(rob.position);
            viz.drawRobot(&rob);

            // if (i % 10 == 0) {
            //     viz.update();
            // }
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
