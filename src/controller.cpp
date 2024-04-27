#define SCALING_FACTOR 5
#include <stdexcept>

#include "types.h"

class Controller {
public:
    virtual fixedVector control(fixedVector sensorValues) = 0;
};

class BraitenbergAgressor : public Controller {
public:
     fixedVector control(fixedVector sensorValues) {
        return {sensorValues[1] * SCALING_FACTOR, sensorValues[0] * SCALING_FACTOR};
     }
};

class BraitenbergFear : public Controller {
public:
     fixedVector control(fixedVector sensorValues) {
        return {sensorValues[0] * SCALING_FACTOR, sensorValues[1] * SCALING_FACTOR};
     }
};

Controller* buildController(std::string identifier) {
    if (identifier == "aggressor") {
        return new BraitenbergAgressor();
    } else if (identifier == "fear") {
        return new BraitenbergFear();
    } else {
        throw std::invalid_argument("Controller " + identifier + " unknown.");
    }
}