#include <vector>

#include "types.h"

/**
 * @brief Convert an angle to a vector in two dimensional space.
 * 
 * @param angle Angle in degrees.
 * @param vectorLength Length the vector should have.
 * 
 * @return Vector relative to the [0, 0] vector with the given length.
 */
fixedVector angleToVector(double angle, double vectorLength);

/**
 * @brief Check whether two values are nearly equal to each other.
 * 
 * @param a First value.
 * @param b Second value.
 * 
 * @return Whether the values are equal to a certain decimal position.
 */
bool isNearlyEqual(double a, double b);

/**
 * @brief Get the Euclidean distance between two points.
 * 
 * @param x1 X of point 1.
 * @param y1 Y of point 1.
 * @param x2 X of point 2.
 * @param y2 Y of point 2.
 * 
 * @return Distance between the points. 
 */
double calcLength(double x1, double y1, double x2, double y2);