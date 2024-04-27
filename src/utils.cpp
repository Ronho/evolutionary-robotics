#define _USE_MATH_DEFINES
#define ERROR_BOUND 1e-5

#include <vector>
#include <cmath>

std::vector<double> angleToVector(double angle, double vectorLength = 1) {
    double x = std::cos(angle/180 * M_PI) * vectorLength/2;
    double y = std::sin(angle/180 * M_PI) * vectorLength/2;
    return std::vector<double>{x, y};
}

bool isNearlyEqual(double a, double b) {
    return std::abs(a-b) < ERROR_BOUND;
}