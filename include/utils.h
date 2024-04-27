#include <vector>

#include "types.h"

// Convert an angle to a vector in two dimensional space.
//
// Args:
// - angle: Angle in degrees.
// - vectorLength: Length the vector should have.
//
// Returns:
//   Vector relative to the [0, 0] vector with the given length.
fixedVector angleToVector(double angle, double vectorLength);

bool isNearlyEqual(double a, double b);

double calcLength(double x1, double y1, double x2, double y2);