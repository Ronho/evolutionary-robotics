#include <vector>
#include <stdexcept>

class Controller {
public:
    virtual std::vector<double> control(std::vector<double> sensorValues);
};

class BraitenbergAgressor : public Controller {
public:
     std::vector<double> control(std::vector<double> sensorValues);
};

class BraitenbergFear : public Controller {
public:
     std::vector<double> control(std::vector<double> sensorValues);
};

Controller* buildController(std::string identifier);