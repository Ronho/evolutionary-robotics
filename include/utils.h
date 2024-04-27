#include <vector>

// Convert an angle to a vector in two dimensional space.
//
// Args:
// - angle: Angle in degrees.
// - vectorLength: Length the vector should have.
//
// Returns:
//   Vector relative to the [0, 0] vector with the given length.
std::vector<double> angleToVector(double angle, double vectorLength);

bool isNearlyEqual(double a, double b);