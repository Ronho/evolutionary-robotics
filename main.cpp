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
#include "map.h"
#include "types.h"
#include "light.h"

class Robot {
public:
    fixedVector position;
    double heading;
    double radius;
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
            double rotationOffset = (this->radius) * (speed[1] + speed[0])/(speed[1] - speed[0]);
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
            throw std::invalid_argument("Index out of bounds!");
        }

        return angleToVector(this->heading + this->sensors[idx], this->radius);
    }

    fixedVector getSensorPosition(int idx) {
        fixedVector relativeVec = this->getRelativeSensorVector(idx);
        return {this->position[0] + relativeVec[0], this->position[1] + relativeVec[1]};
    }
};

class ProximityRobot {
public:
    fixedVector position;
    double heading;
    double radius;
    std::vector<double> sensors;

public:
    ProximityRobot(fixedVector position, double heading, double radius, std::vector<double> sensors) {
        this->position = position;
        this->heading = heading;
        this->radius = radius;
        this->sensors = sensors;
    }
    void drive(fixedVector speed, double timeSteps) {
        fixedVector newPosition;
        double rotationAngle;
        if (isNearlyEqual(speed[0], speed[1])) {
            // Straight forward.
            rotationAngle = 0;
            newPosition = angleToVector(this->heading, speed[1] * timeSteps);
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

    void draw(matplot::axes_handle ax) {
        ax->ellipse(this->position[0]-this->radius, this->position[1]-this->radius, this->radius * 2, this->radius * 2);
        fixedVector vec = this->getHeadingVector();
        matplot::vectors_handle v = ax->arrow(this->position[0], this->position[1], this->position[0] + vec[0], this->position[1] + vec[1]);
        v->color("red");
        for (int i = 0; i < this->sensors.size(); i++) {
            vec = this->getRelativeSensorVector(i);
            ax->arrow(this->position[0], this->position[1], this->position[0] + vec[0], this->position[1] + vec[1]);
        }
    }

    // TODO
    fixedVector getHeadingVector() {
        return angleToVector(this->heading, this->radius);
    }

    // TODO
    fixedVector getRelativeSensorVector(int idx) {
        if (idx >= this->sensors.size()) {
            throw std::invalid_argument("Index out of bounds!");
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
public:
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
        v = this->drawArrowRelativeToCenter(rob->position, rob->getRelativeSensorVector(0));
        v->color("blue");
        this->drawArrowRelativeToCenter(rob->position, rob->getRelativeSensorVector(1));
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

void scenario1() {
    Torus map = Torus({-100, 100}, {-100, 100});
    Controller* controller = buildController("fear");
    Robot rob({-90, -90}, 90, 5.0, {45, -45});
    Light buzz({40, 40}, 10);
    double norm = calcLength(map.xlim[0], map.ylim[0], map.xlim[1], map.ylim[1]);
    Visualizer viz(map.xlim, map.ylim);

    try {
        buzz.draw(viz.ax);
        viz.drawRobot(&rob);
        map.draw(viz.ax);
        viz.update();
        for (int i = 0; i < 10; i++) {
            #ifdef DEBUG
                std::cout << "Iteration: " << i << " - Position: " << rob.position[0] << "," << rob.position[1] << std::endl;
            #endif

            fixedVector sensedValues = {buzz.getIntensity(rob.getSensorPosition(0), norm), buzz.getIntensity(rob.getSensorPosition(1), norm)};
            rob.drive(controller->control(sensedValues), 1);
            rob.position = map.clip(rob.position);
            viz.drawRobot(&rob);

            // if (i % 10 == 0) {
            //     viz.update();
            // }
        }
        viz.save("path_scenario_1.png");
        viz.show();
    } catch (const std::runtime_error &e) {
        // If you encounter the "popen() failed!" error, you likely have not
        // installed gnuplot correctly.
        std::cerr << "Runtime error: " << e.what() << std::endl;
    }
}

double proximity(fixedVector sensorBeginning, fixedVector sensorEnd, Bounded* map) {
    double shortestDistance = -1;
    double distance = -1;
    double scalingFactor = -1;
    double x = -1;
    double y = -1;
    // Check bounds.
    // Assuming that the beginning has to be inside the bounds.
    if (sensorEnd[0] < map->xlim[0]) {
        x = map->xlim[0];
        scalingFactor = (x - sensorBeginning[0])/ (sensorEnd[0] - sensorBeginning[0]);
        y = scalingFactor * (sensorEnd[1] - sensorBeginning[1]) + sensorBeginning[1];
        if (x != -1 && y != -1) {
            distance = calcLength(x, y, sensorBeginning[0], sensorBeginning[1]);
            if (shortestDistance == -1 || distance < shortestDistance) {
                shortestDistance = distance;
            }
        }
    }
    if (sensorEnd[0] > map->xlim[1]) {
        x = map->xlim[1];
        scalingFactor = (x - sensorBeginning[0])/ (sensorEnd[0] - sensorBeginning[0]);
        y = scalingFactor * (sensorEnd[1] - sensorBeginning[1]) + sensorBeginning[1];
        if (x != -1 && y != -1) {
            distance = calcLength(x, y, sensorBeginning[0], sensorBeginning[1]);
            if (shortestDistance == -1 || distance < shortestDistance) {
                shortestDistance = distance;
            }
        }
    }
    if (sensorEnd[1] < map->ylim[0]) {
        y = map->ylim[0];
        scalingFactor = (y - sensorBeginning[1])/ (sensorEnd[1] - sensorBeginning[1]);
        x = scalingFactor * (sensorEnd[0] - sensorBeginning[0]) + sensorBeginning[0];
        if (x != -1 && y != -1) {
            distance = calcLength(x, y, sensorBeginning[0], sensorBeginning[1]);
            if (shortestDistance == -1 || distance < shortestDistance) {
                shortestDistance = distance;
            }
        }
    }
    if (sensorEnd[1] > map->ylim[1]) {
        y = map->ylim[1];
        scalingFactor = (y - sensorBeginning[1])/ (sensorEnd[1] - sensorBeginning[1]);
        x = scalingFactor * (sensorEnd[0] - sensorBeginning[0]) + sensorBeginning[0];
        if (x != -1 && y != -1) {
            distance = calcLength(x, y, sensorBeginning[0], sensorBeginning[1]);
            if (shortestDistance == -1 || distance < shortestDistance) {
                shortestDistance = distance;
            }
        }
    }

    // Check walls following https://stackoverflow.com/questions/99353/how-to-test-if-a-line-segment-intersects-an-axis-aligned-rectange-in-2d.


    Wall *wall;
    double slopeProximity = (sensorEnd[1]-sensorBeginning[1]) / (sensorEnd[0]-sensorBeginning[0]); // Could be inf if parallel to y-axis.
    double interceptProximity = sensorBeginning[1] - slopeProximity  * sensorBeginning[0];
    double lower = std::min(sensorBeginning[1], sensorEnd[1]);
    double upper = std::max(sensorBeginning[1], sensorEnd[1]);
    double left = std::min(sensorBeginning[0], sensorEnd[0]);
    double right = std::max(sensorBeginning[0], sensorEnd[0]);
    double slopeWall, intersectionX, intersectionY;
    for (int i = 0; i < map->walls.size(); i++) {
        wall = &map->walls[i];

        // Check if any outer line of the wall intersects with the given line.
        // Top-Left -> Top-Right
        slopeWall = 0;
        if (!isNearlyEqual(slopeWall, slopeProximity)) {
            // At which point do they intersect?
            intersectionY = wall->y2;
            // Is the point within the bounds of the sensor?
            if (intersectionY < upper && intersectionY > lower) {
                intersectionX = (intersectionY-interceptProximity)/slopeProximity;

                // Is the point within the bounds of the rectangle?
                if (intersectionX < wall->x2 && intersectionX > wall->x1) {
                    // How far is the point?
                    distance = calcLength(intersectionX, intersectionY, sensorBeginning[0], sensorBeginning[1]);
                    if (shortestDistance == -1 || distance < shortestDistance) {
                        shortestDistance = distance;
                    }
                }
            }
        } // Else: Lines are parallel. For simplicity, we do not check whether they are overlapping.

        // Top-Right -> Bottom-Right
        slopeWall = std::numeric_limits<double>::infinity();
        if (!isNearlyEqual(slopeWall, slopeProximity)) {
            // At which point do they intersect?
            intersectionX = wall->x2;
            // Is the point within the bounds?
            if (intersectionX < right && intersectionX > left) {
                intersectionY = slopeProximity*intersectionX+interceptProximity;
                
                // Is the point within the bounds of the rectangle?
                if (intersectionY < wall->y2 && intersectionY > wall->y1) {
                    // How far is the point?
                    distance = calcLength(intersectionX, intersectionY, sensorBeginning[0], sensorBeginning[1]);
                    if (shortestDistance == -1 || distance < shortestDistance) {
                        shortestDistance = distance;
                    }
                }
            }
        } // Else: Lines are parallel. For simplicity, we do not check whether they are overlapping.

        // Bottom-Right -> Bottom-Left
        slopeWall = 0;
        if (!isNearlyEqual(slopeWall, slopeProximity)) {
            // At which point do they intersect?
            intersectionY = wall->y1;
            // Is the point within the bounds of the sensor?
            if (intersectionY < upper && intersectionY > lower) {
                intersectionX = (intersectionY-interceptProximity)/slopeProximity;

                // Is the point within the bounds of the rectangle?
                if (intersectionX < wall->x2 && intersectionX > wall->x1) {
                    // How far is the point?
                    distance = calcLength(intersectionX, intersectionY, sensorBeginning[0], sensorBeginning[1]);
                    if (shortestDistance == -1 || distance < shortestDistance) {
                        shortestDistance = distance;
                    }
                }
            }
        } // Else: Lines are parallel. For simplicity, we do not check whether they are overlapping.

        // Bottom-Left -> Top-Left
        slopeWall = std::numeric_limits<double>::infinity();
        if (!isNearlyEqual(slopeWall, slopeProximity)) {
            // At which point do they intersect?
            intersectionX = wall->x1;
            // Is the point within the bounds?
            if (intersectionX < right && intersectionX > left) {
                intersectionY = slopeProximity*intersectionX+interceptProximity;
                
                // Is the point within the bounds of the rectangle?
                if (intersectionY < wall->y2 && intersectionY > wall->y1) {
                    // How far is the point?
                    distance = calcLength(intersectionX, intersectionY, sensorBeginning[0], sensorBeginning[1]);
                    if (shortestDistance == -1 || distance < shortestDistance) {
                        shortestDistance = distance;
                    }
                }
            }
        } // Else: Lines are parallel. For simplicity, we do not check whether they are overlapping.
    }
    return shortestDistance;
}

fixedVector proximityController(std::vector<double> values) {
    // Sensed values will be bigger with closer obstacles.
    // Values above 1 or below 0 are measurement errors (e.g., there is no obstacle).

    if (values[1] > 1 || values[1] < 0.5) {
        std::cout << "Straight" << std::endl;
        // Nothing in front. Just walk straight.
        return {5, 5};
    }

    // Something is in front of us. Can we go left?
    if (values[0] > 1 || values[0] < 0.5) {
        std::cout << "Left" << std::endl;
        return {-3, 7};
    }

    // Something is in front and left of us. Turn right.
    std::cout << "Right" << std::endl;
    return {7, -3};
}

void scenario2() {
    std::vector<Wall> walls;
    walls.push_back(Wall(-100, -2, -50, 2));
    walls.push_back(Wall(-2, -100, 2, 40));
    walls.push_back(Wall(50, 68, 100, 72));
    Bounded map({-100, 100}, {-100, 100}, walls);
    ProximityRobot rob({-90, -90}, 0, 5.0, {22.5, 0, -22.5});
    Visualizer viz(map.xlim, map.ylim);

    std::vector<double> sensedValues = {-1, -1, -1}; // -1 meaning no object detected.
    fixedVector sensorEnd = {-1, -1};
    const double range = (map.ylim[1] - map.ylim[0]) * 0.15;
    try {
        rob.draw(viz.ax);
        map.draw(viz.ax);
        viz.update();

        for (int i = 0; i < 1000; i++) {
            #ifdef DEBUG
                std::cout << "Iteration: " << i << " - Position: " << rob.position[0] << "," << rob.position[1] << std::endl;
                std::cout << "Sensed: " << sensedValues[0] << ", " << sensedValues[1] << ", " << sensedValues[2] << std::endl;
            #endif

            for (int j = 0; j < rob.sensors.size(); j++) {
                sensorEnd = angleToVector(rob.heading + rob.sensors[j], range);
                sensorEnd = {rob.position[0] + sensorEnd[0], rob.position[1] + sensorEnd[1]};
                sensedValues[j] = 1-(proximity(rob.position, sensorEnd, &map)/range);
            }
            rob.drive(proximityController(sensedValues), 1);
            rob.position = map.clip(rob.position); // Collision detection with walls is ommitted.
            rob.draw(viz.ax);
        }

        viz.save("path_scenario_2.png");
        viz.show();
    } catch (const std::runtime_error &e) {
        // If you encounter the "popen() failed!" error, you likely have not
        // installed gnuplot correctly.
        std::cerr << "Runtime error: " << e.what() << std::endl;
    }
}

int main(int argc, char* argv[]) {
    std::cout << "You are running version " << EvoRob_VERSION_MAJOR << "." << EvoRob_VERSION_MINOR << "." << std::endl;
    std::string selectedScenario = "light";

    if (argc > 1) {
        selectedScenario = argv[1];
    }
    
    if (selectedScenario == "light") {
        scenario1();
    } else if (selectedScenario == "proximity") {
        scenario2();
    } else {
        std::cerr << "Invalid argument: " << selectedScenario << "." << std::endl;
    }

    return 0;
}
