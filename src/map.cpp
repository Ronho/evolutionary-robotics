#include <vector>
#include <matplot/matplot.h>

#include "types.h"
#include "map.h"

Torus::Torus(fixedVector xlim, fixedVector ylim) {
    this->xlim = xlim;
    this->ylim = ylim;
}

fixedVector Torus::clip(fixedVector position) {
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

void Torus::draw(matplot::axes_handle ax) {
    ax->rectangle(this->xlim[0], this->ylim[0], this->xlim[1] - this->xlim[0], this->ylim[1] - this->ylim[0]);
}

Wall::Wall(double x1, double y1, double x2, double y2) {
    this->x1 = x1;
    this->x2 = x2;
    this->y1 = y1;
    this->y2 = y2;
}
double Wall::getWidth() {
    return this->x2 - this->x1;
}

double Wall::getHeight() {
    return this->y2 - this->y1;
}

bool Wall::within(int x, int y) {
    return (x >= this->x1) && (x <= this->x2) && (y >= this->y1) && (y <= this->y2);
}

Bounded::Bounded(fixedVector xlim, fixedVector ylim, std::vector<Wall> walls) {
    this->xlim = xlim;
    this->ylim = ylim;
    this->walls = walls;
}

fixedVector Bounded::clip(fixedVector position) {
    position[0] = std::min(this->xlim[1], position[0]);
    position[0] = std::max(this->xlim[0], position[0]);
    position[1] = std::min(this->ylim[1], position[1]);
    position[1] = std::max(this->ylim[0], position[1]);
    return position;
}

void Bounded::draw(matplot::axes_handle ax) {
    ax->rectangle(this->xlim[0], this->ylim[0], this->xlim[1] - this->xlim[0], this->ylim[1] - this->ylim[0]);
    Wall* wall;
    for(int i = 0; i < this->walls.size(); i++) {
        wall = &this->walls[i];
        ax->rectangle(wall->x1, wall->y1, wall->getWidth(), wall->getHeight());
    }
}