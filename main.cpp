#define _USE_MATH_DEFINES

#include <cmath>
#include <iostream>
#include <matplot/matplot.h>
#include <random>
#include <set>
#include <stdexcept>
#include <vector>

#include "config.h"
#include "controller.h"
#include "light.h"
#include "map.h"
#include "robot.h"
#include "types.h"
#include "utils.h"

/**
 * @brief Calculate the proximity of the closest object observed by the sensor.
 * 
 * Sensed values will be bigger with closer obstacles. Values above 1 or below 0
 * are measurement errors (no obstacle detected).
 * 
 * @param sensorBeginning Starting point of the sensor. Usually equal to the
 * robots center.
 * @param sensorEnd End point of the sensor.
 * @param map A bounded map.
 * 
 * @return Proximity of the closest object in range.
 */
double proximity(fixedVector sensorBeginning, fixedVector sensorEnd, Bounded* map) {
    double shortestDistance = -1;
    double distance = -1;
    double scalingFactor = -1;
    double x = -1;
    double y = -1;

    // Check bounds of the map. (Does the sensor point to an end of the map?)
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


    Wall *wall;
    double slopeProximity;
    if (isNearlyEqual(sensorEnd[0]-sensorBeginning[0], 0)) {
        slopeProximity = 1;
    } else {
        // Could be inf if parallel to y-axis (covered by case before).
        slopeProximity = (sensorEnd[1]-sensorBeginning[1]) / (sensorEnd[0]-sensorBeginning[0]);
    }
    double interceptProximity = sensorBeginning[1] - slopeProximity  * sensorBeginning[0];
    double lower = std::min(sensorBeginning[1], sensorEnd[1]);
    double upper = std::max(sensorBeginning[1], sensorEnd[1]);
    double left = std::min(sensorBeginning[0], sensorEnd[0]);
    double right = std::max(sensorBeginning[0], sensorEnd[0]);
    double slopeWall, intersectionX, intersectionY;

    // For each wall check if the any of its edge of it has an intersection with
    // the line segment created by the sensor. If so, the sensor can see this
    // wall.
    for (int i = 0; i < map->walls.size(); i++) {
        wall = &map->walls[i];

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

void lightScenario(std::string selectedController) {
    Torus map = Torus({-100, 100}, {-100, 100});
    LightController* controller = buildLightController(selectedController);
    Robot rob({-90, -90}, 90, 5.0, {45, -45});
    Light buzz({40, 40}, 10, calcLength(map.xlim[0], map.ylim[0], map.xlim[1], map.ylim[1]));

    std::vector<double> sensedValues;
    fixedVector speeds;
    try {
        matplot::figure_handle f = matplot::figure(true);
        matplot::axes_handle ax = f->current_axes();
        ax->xlim(map.xlim);
        ax->ylim(map.ylim);

        buzz.draw(ax);
        rob.draw(ax);
        map.draw(ax);
        f->draw();
        for (int i = 0; i < 1000; i++) {
            sensedValues = {buzz.getIntensity(rob.getSensorPosition(0)), buzz.getIntensity(rob.getSensorPosition(1))};
            speeds = controller->control(sensedValues);

            #ifdef DEBUG
                std::cout << "Iteration: " << i << " - Position: " << rob.position[0] << "," << rob.position[1] << std::endl;
                std::cout << "Sensed: " << sensedValues[0] << "," << sensedValues[1] << std::endl;
                std::cout << "Speeds: " << speeds[0] << "," << speeds[1] << std::endl;
            #endif

            rob.drive(speeds, 1);
            rob.position = map.clip(rob.position);
            rob.draw(ax);
        }
        f->save("light_scenario_trajectory.png");
        f->show();
    } catch (const std::runtime_error &e) {
        // If you encounter the "popen() failed!" error, you likely have not
        // installed gnuplot correctly.
        std::cerr << "Runtime error: " << e.what() << std::endl;
    }
}

void proximityScenario() {
    std::vector<Wall> walls;
    walls.push_back(Wall(-100, -2, -50, 2));
    walls.push_back(Wall(-2, -100, 2, 40));
    walls.push_back(Wall(50, 68, 100, 72));
    Bounded map({-100, 100}, {-100, 100}, walls);
    Robot rob({-90, -90}, 0, 5.0, {22.5, 0, -22.5});
    HandMadeProxCon con;
    const double range = (map.ylim[1] - map.ylim[0]) * 0.15;

    std::vector<double> sensedValues = {-1, -1, -1}; // -1 = no object detected.
    fixedVector sensorEnd = {-1, -1};
    fixedVector speeds;

    try {
        matplot::figure_handle f = matplot::figure(true);
        matplot::axes_handle ax = f->current_axes();
        ax->xlim(map.xlim);
        ax->ylim(map.ylim);

        rob.draw(ax);
        map.draw(ax);
        f->draw();

        for (int i = 0; i < 1000; i++) {
            for (int j = 0; j < rob.sensors.size(); j++) {
                sensorEnd = angleToVector(rob.heading + rob.sensors[j], range);
                sensorEnd = {
                    rob.position[0] + sensorEnd[0],
                    rob.position[1] + sensorEnd[1]
                };
                sensedValues[j] = 1 - (
                    proximity(rob.position, sensorEnd, &map) / range
                );
            }
            speeds = con.control(sensedValues);

            #ifdef DEBUG
                std::cout << "Iteration: " << i << " - Position: "
                    << rob.position[0] << "," << rob.position[1] << std::endl;
                std::cout << "Sensed: " << sensedValues[0] << ", "
                    << sensedValues[1] << ", " << sensedValues[2] << std::endl;
                std::cout << "Speeds: " << speeds[0] << ", " << speeds[1]
                    << std::endl;
            #endif

            rob.drive(speeds, 1);
            rob.position = map.clip(rob.position);
            rob.draw(ax);
        }

        f->save("proximity_scenario_trajectory.png");
        f->show();
    } catch (const std::runtime_error &e) {
        // If you encounter the "popen() failed!" error, you likely have not
        // installed gnuplot correctly.
        std::cerr << "Runtime error: " << e.what() << std::endl;
    }
}


void hill_climber_scenario() {

    std::vector<Wall> walls;
    walls.push_back(Wall(-100, -2, -50, 2));
    walls.push_back(Wall(-2, -100, 2, 40));
    walls.push_back(Wall(50, 68, 100, 72));
    Bounded map({-100, 100}, {-100, 100}, walls);
    Robot rob({-90, -90}, 0, 5.0, {22.5, 0, -22.5});
    HandMadeProxCon con;
    const double range = (map.ylim[1] - map.ylim[0]) * 0.15;

    std::vector<double> sensedValues = {-1, -1, -1}; // -1 = no object detected.
    fixedVector sensorEnd = {-1, -1};
    fixedVector speeds;
    double vl, vr = 0;

    // Setup for random generation.
    std::uniform_real_distribution<double> contDistribution(-5, 5);
    std::uniform_int_distribution<uint16_t> discDistribution(0, 5);
    std::random_device dev;
    // Make every run different:
    // std::mt19937 engine(dev()); // Mersenne twister MT19937
    std::mt19937 engine; // Random but every time the same.
    auto valueGenerator = std::bind(contDistribution, engine);
    auto locGenerator = std::bind(discDistribution, engine);

    // The genes consists of the slope and intercept for each sensor: y=mx+c.
    // Each pair is next to each other, meaning:
    // [0] = m0, [1] = c0, [2] = m1, [3] = c1, [4] = m2, [5] = c2
    // m0 and c0 correspond to the sensor who generates the sensedValues at
    // index 0: v0 = genes[0] * sensedValues[0] + genes[1]
    std::vector<double> genes = {1, 1, 1, 1, 1, 1}; // TODO: has to be random!
    std::generate(std::begin(genes), std::end(genes), valueGenerator);

    std::vector<double> new_genes = genes;
    std::set<std::pair<int, int>> visited;
    size_t fitness = 0;
    size_t new_fitness = 0;
    uint16_t location = 0;

    for (int epoch = 0; epoch < 10000; epoch++) {
        for (int i = 0; i < 1000; i++) {
            for (int j = 0; j < rob.sensors.size(); j++) {
                sensorEnd = angleToVector(rob.heading + rob.sensors[j], range);
                sensorEnd = {
                    rob.position[0] + sensorEnd[0],
                    rob.position[1] + sensorEnd[1]
                };
                sensedValues[j] = 1 - (
                    proximity(rob.position, sensorEnd, &map) / range
                );
            }
            vl = new_genes.at(0) * sensedValues.at(0) + new_genes.at(1);
            vr = new_genes.at(2) * sensedValues.at(1) + new_genes.at(3)
                 + new_genes.at(4) * sensedValues.at(2) + new_genes.at(5);
            speeds = {vl, vr};

            rob.drive(speeds, 1);
            rob.position = map.clip(rob.position);

            std::pair<int, int> x = std::make_pair(static_cast<int>(std::ceil(rob.position.at(0))), static_cast<int>(std::ceil(rob.position.at(1))));
            visited.insert(x);
        }
        new_fitness = visited.size();
        visited = {};
        std::cout << "New fitness: " << new_fitness << std::endl;

        if (new_fitness > fitness) {
            fitness = new_fitness;
            genes = new_genes;
        }

        std::cout << "Best fitness: " << fitness << std::endl;

        // Mutate:
        // First, choose one value to change.
        // Second, choose a new value.
        location = locGenerator();
        new_genes.at(location) = valueGenerator();
    }

    try {
        matplot::figure_handle f = matplot::figure(true);
        matplot::axes_handle ax = f->current_axes();
        ax->xlim(map.xlim);
        ax->ylim(map.ylim);

        rob.draw(ax);
        map.draw(ax);
        f->draw();

        for (int i = 0; i < 1000; i++) {
            for (int j = 0; j < rob.sensors.size(); j++) {
                sensorEnd = angleToVector(rob.heading + rob.sensors[j], range);
                sensorEnd = {
                    rob.position[0] + sensorEnd[0],
                    rob.position[1] + sensorEnd[1]
                };
                sensedValues[j] = 1 - (
                    proximity(rob.position, sensorEnd, &map) / range
                );
            }
            vl = genes.at(0) * sensedValues.at(0) + genes.at(1);
            vr = genes.at(2) * sensedValues.at(1) + genes.at(3)
                 + genes.at(4) * sensedValues.at(2) + genes.at(5);
            speeds = {vl, vr};

            rob.drive(speeds, 1);
            rob.position = map.clip(rob.position);
            rob.draw(ax);

            std::pair<int, int> x = std::make_pair(static_cast<int>(std::ceil(rob.position.at(0))), static_cast<int>(std::ceil(rob.position.at(1))));
            visited.insert(x);
        }
        fitness = visited.size();

        f->save("hill_climber_trajectory.png");
        f->show();
    } catch (const std::runtime_error &e) {
        // If you encounter the "popen() failed!" error, you likely have not
        // installed gnuplot correctly.
        std::cerr << "Runtime error: " << e.what() << std::endl;
    }
}

int main(int argc, char* argv[]) {
    std::cout << "You are running version " << EvoRob_VERSION_MAJOR << "."
        << EvoRob_VERSION_MINOR << "." << std::endl;
    
    // Fallback values.
    std::string selectedScenario = "hill";
    std::string selectedController = "aggressor";

    if (argc > 1) {
        selectedScenario = argv[1];
    }
    if (argc > 2) {
        selectedController = argv[2];
    }
    
    if (selectedScenario == "light") {
        lightScenario(selectedController);
    } else if (selectedScenario == "proximity") {
        proximityScenario();
    } else if (selectedScenario == "hill") {
        hill_climber_scenario();
    } else {
        std::cerr << "Invalid argument: " << selectedScenario << "."
            << std::endl;
    }

    return 0;
}
