#include <vector>
#include <stdexcept>

#include "types.h"

class Controller {
public:
    virtual fixedVector control(fixedVector sensorValues);
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