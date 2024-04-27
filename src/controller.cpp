#include <stdexcept>

#include "types.h"

class Controller {
public:
    virtual fixedVector control(fixedVector sensorValues) = 0;
};

class BraitenbergAgressor : public Controller {
public:
     fixedVector control(fixedVector sensorValues) {
        return {1, 1};
     }
};

class BraitenbergFear : public Controller {
public:
     fixedVector control(fixedVector sensorValues) {
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