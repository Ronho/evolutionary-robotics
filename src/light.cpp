#include <matplot/matplot.h>

#include "types.h"
#include "utils.h"
#include "light.h"

Light::Light(fixedVector position, double radius, double norm) {
    this->position = position;
    this->radius = radius;
    this->norm = norm;
}

double Light::getIntensity(fixedVector sensorPos) {
    return 1 - (calcLength(sensorPos[0], sensorPos[1], this->position[0], this->position[1]) / this->norm);
}

void Light::draw(matplot::axes_handle ax) {
    auto a = ax->ellipse(this->position[0]-this->radius, this->position[1]-this->radius, this->radius * 2, this->radius * 2);
    a->fill(true);
    a->color("green");
}