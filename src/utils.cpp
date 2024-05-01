#define _USE_MATH_DEFINES
#define ERROR_BOUND 1e-5

#include <vector>
#include <cmath>

#include "types.h"
#include "utils.h"

fixedVector angleToVector(double angle, double vectorLength = 1) {
    double x = std::cos(angle/180 * M_PI) * vectorLength;
    double y = std::sin(angle/180 * M_PI) * vectorLength;
    return fixedVector{x, y};
}

bool isNearlyEqual(double a, double b) {
    return std::abs(a-b) < ERROR_BOUND;
}

double calcLength(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}