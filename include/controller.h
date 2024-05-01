#include <vector>
#include <stdexcept>

#include "types.h"

class Controller {
public:
    virtual fixedVector control(fixedVector sensorValues) = 0;
};

class BraitenbergAgressor : public Controller {
public:
     fixedVector control(fixedVector sensorValues);
};

class BraitenbergFear : public Controller {
public:
     fixedVector control(fixedVector sensorValues);
};

Controller* buildController(std::string identifier);