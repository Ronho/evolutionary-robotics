#include <stdexcept>
#include <vector>
#include <matplot/matplot.h>

#include "types.h"

class Map {
public:
    fixedVector xlim;
    fixedVector ylim;

public:
    virtual fixedVector clip(fixedVector position) = 0;
    virtual void draw(matplot::axes_handle ax) = 0;
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

    void draw(matplot::axes_handle ax) {
        ax->rectangle(this->xlim[0], this->ylim[0], this->xlim[1] - this->xlim[0], this->ylim[1] - this->ylim[0]);
    }
};

// Assumption: 1 is lower left corner, 2 is upper right corner.
class Wall {
public:
    double x1;
    double x2;
    double y1;
    double y2;

public:
    Wall(double x1, double y1, double x2, double y2) {
        this->x1 = x1;
        this->x2 = x2;
        this->y1 = y1;
        this->y2 = y2;
    }

    double getWidth() {
        return this->x2 - this->x1;
    }

    double getHeight() {
        return this->y2 - this->y1;
    }
};

class Bounded : public Map {
public:
    std::vector<Wall> walls;
public:
    Bounded(fixedVector xlim, fixedVector ylim, std::vector<Wall> walls) {
        this->xlim = xlim;
        this->ylim = ylim;
        this->walls = walls;
    }

    fixedVector clip(fixedVector position) {
        position[0] = std::min(this->xlim[1], position[0]);
        position[0] = std::max(this->xlim[0], position[0]);
        position[1] = std::min(this->ylim[1], position[1]);
        position[1] = std::max(this->ylim[0], position[1]);
        return position;
    }

    void draw(matplot::axes_handle ax) {
        ax->rectangle(this->xlim[0], this->ylim[0], this->xlim[1] - this->xlim[0], this->ylim[1] - this->ylim[0]);
        Wall* wall;
        for(int i = 0; i < this->walls.size(); i++) {
            wall = &this->walls[i];
            ax->rectangle(wall->x1, wall->y1, wall->getWidth(), wall->getHeight());
        }
    }
};

Map* buildMap(std::string identifier) {
    if (identifier == "torus") {
        return new Torus({-100, 100}, {-100, 100});
    } else if (identifier == "bounded") {
        std::vector<Wall> walls;
        walls.push_back(Wall(-100, -2, -50, 2));
        walls.push_back(Wall(-2, -100, 2, 40));
        walls.push_back(Wall(50, 68, 100, 72));
        return new Bounded({-100, 100}, {-100, 100}, walls);
    } else {
        throw std::invalid_argument("Map " + identifier + " unknown.");
    }
}