#include <stdexcept>

#include "types.h"

class Map {
public:
    fixedVector xlim;
    fixedVector ylim;

public:
    virtual fixedVector clip(fixedVector position) = 0;
};

class Torus : public Map {
public:
    Torus(fixedVector xlim, fixedVector ylim) {
        this->xlim = xlim;
        this->ylim = ylim;
    }

    fixedVector clip(fixedVector position) {
        if (position[0] > this->xlim[1]) {
            position[0] -= (this->xlim[1] - this->xlim[0]);
        } else if (position[0] < xlim[0]) {
            position[0] += (this->xlim[1] - this->xlim[0]);
        }
        if (position[1] > this->ylim[1]) {
            position[1] -= (this->ylim[1] - this->ylim[0]);
        } else if (position[1] < ylim[0]) {
            position[1] += (this->ylim[1] - this->ylim[0]);
        }
        return position;
    }
};

class Bounded : public Map {
public:
    Bounded(fixedVector xlim, fixedVector ylim) {
        this->xlim = xlim;
        this->ylim = ylim;
    }

    fixedVector clip(fixedVector position) {
        position[0] = std::min(this->xlim[1], position[0]);
        position[0] = std::max(this->xlim[0], position[0]);
        position[1] = std::min(this->ylim[1], position[0]);
        position[1] = std::max(this->ylim[0], position[0]);
        return position;
    }
};

Map* buildMap(std::string identifier) {
    if (identifier == "torus") {
        return new Torus({-100, 100}, {-100, 100});
    } else if (identifier == "bounded") {
        return new Bounded({-100, 100}, {-100, 100});
    } else {
        throw std::invalid_argument("Controller " + identifier + " unknown.");
    }
}