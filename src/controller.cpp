#define SCALING_FACTOR 5
#include <stdexcept>

#include "types.h"
#include "controller.h"

fixedVector BraitenbergAgressor::control(std::vector<double> sensorValues) {
    return {sensorValues[1] * SCALING_FACTOR, sensorValues[0] * SCALING_FACTOR};
}

fixedVector BraitenbergFear::control(std::vector<double> sensorValues) {
    return {sensorValues[0] * SCALING_FACTOR, sensorValues[1] * SCALING_FACTOR};
}

LightController* buildLightController(std::string identifier) {
    if (identifier == "aggressor") {
        return new BraitenbergAgressor();
    } else if (identifier == "fear") {
        return new BraitenbergFear();
    } else {
        throw std::invalid_argument("Controller " + identifier + " unknown.");
    }
}

fixedVector HandMadeProxCon::control(std::vector<double> sensorValues) {
    if (sensorValues[1] > 1 || sensorValues[1] < 0.5) {
        // Nothing in front. Just go straight.
        return {5, 5};
    }

    // Something is in front of us. Can we go left?
    if (sensorValues[0] > 1 || sensorValues[0] < 0.5) {
        return {-3, 3};
    }

    // Something is in front and left of us. Turn right.
    return {3, -3};
}