#define _USE_MATH_DEFINES

#include <cmath>
#include <stdexcept>
#include <vector>

#include "robot.h"
#include "utils.h"

Robot::Robot(fixedVector position, double heading, double radius, std::vector<double> sensors) {
    this->position = position;
    this->heading = heading;
    this->radius = radius;
    this->sensors = sensors;
}

void Robot::drive(fixedVector speed, double timeSteps, std::string mode) {
    if (mode == "exact") {
        fixedVector newPositionOffset;
        double rotationAngle;
        if (isNearlyEqual(speed[0], speed[1])) {
            // Straight forward.
            rotationAngle = 0;
            newPositionOffset = angleToVector(this->heading, speed[1] * timeSteps);
        } else if (isNearlyEqual(-speed[0], speed[1])) {
            // Rotate on spot.
            rotationAngle = (360 * speed[1] * timeSteps)/(2 * M_PI * this->radius);
            newPositionOffset = {0, 0};
        } else {
            double rotationOffset = (this->radius) * (speed[1] + speed[0])/(speed[1] - speed[0]);
            rotationAngle = (360 * speed[0] * timeSteps)/(2 * M_PI * rotationOffset);
            const double headingAngleOffset = (180 - rotationAngle) / 2;
            const double headingLength = rotationOffset / std::tan(headingAngleOffset * M_PI/180);
            newPositionOffset = angleToVector(this->heading + rotationAngle, headingLength);
        }

        this->position[0] += newPositionOffset[0];
        this->position[1] += newPositionOffset[1];

        this->heading += rotationAngle;
    } else if (mode == "approx") {
        double rotationAngle = 3 * (speed[1] - speed[0]);
        this->heading += rotationAngle;

        double meanSpeed = (speed[0] + speed[1]) / 2;
        fixedVector newPositionOffset = angleToVector(this->heading, meanSpeed * timeSteps);
        this->position[0] += newPositionOffset[0];
        this->position[1] += newPositionOffset[1];
    } else {
        throw std::invalid_argument("Mode " + mode + " unknown.");
    }
}

void Robot::draw(matplot::axes_handle ax, bool only_trajectory) {
    fixedVector vec = this->getHeadingVector();
    matplot::vectors_handle v = ax->arrow(this->position[0], this->position[1], this->position[0] + vec[0], this->position[1] + vec[1]);
    v->color("red");
    if (!only_trajectory) {
        ax->ellipse(this->position[0]-this->radius, this->position[1]-this->radius, this->radius * 2, this->radius * 2);
        for (int i = 0; i < this->sensors.size(); i++) {
            vec = this->getRelativeSensorVector(i);
            ax->arrow(this->position[0], this->position[1], this->position[0] + vec[0], this->position[1] + vec[1]);
        }
    }
}

fixedVector Robot::getHeadingVector() {
    return angleToVector(this->heading, this->radius);
}

fixedVector Robot::getRelativeSensorVector(int idx) {
    if (idx >= this->sensors.size()) {
        throw std::invalid_argument("Index out of bounds!");
    }

    return angleToVector(this->heading + this->sensors[idx], this->radius);
}

fixedVector Robot::getSensorPosition(int idx) {
    fixedVector relativeVec = this->getRelativeSensorVector(idx);
    return {this->position[0] + relativeVec[0], this->position[1] + relativeVec[1]};
}