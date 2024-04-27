#include <vector>
#include <stdexcept>

class Controller {
public:
    virtual std::vector<double> control(std::vector<double> sensorValues) = 0;
};

class BraitenbergAgressor : public Controller {
public:
     std::vector<double> control(std::vector<double> sensorValues) {
        return {1, 1};
     }
};

class BraitenbergFear : public Controller {
public:
     std::vector<double> control(std::vector<double> sensorValues) {
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