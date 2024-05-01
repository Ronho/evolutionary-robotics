#include "types.h"
#include "utils.h"
#include "light.h"

// Assumption: Light reaches every point in the field.
Light::Light(fixedVector position, double radius) {
    this->position = position;
    this->radius = radius;
}

double Light::getIntensity(fixedVector sensor, double norm) {
    return 1 - (calcLength(sensor[0], sensor[1], this->position[0], this->position[1]) / norm);
}