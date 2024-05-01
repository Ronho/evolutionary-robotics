#include <matplot/matplot.h>

#include "types.h"
#include "utils.h"
#include "light.h"

Light::Light(fixedVector position, double radius) {
    this->position = position;
    this->radius = radius;
}

double Light::getIntensity(fixedVector sensor, double norm) {
    return 1 - (calcLength(sensor[0], sensor[1], this->position[0], this->position[1]) / norm);
}

void Light::draw(matplot::axes_handle ax) {
    auto a = ax->ellipse(this->position[0]-this->radius, this->position[1]-this->radius, this->radius * 2, this->radius * 2);
    a->fill(true);
    a->color("green");
}