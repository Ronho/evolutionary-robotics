#include <matplot/matplot.h>
#include "types.h"

// Assumption: Light reaches every point in the field.
class Light {
public:
    fixedVector position;
    double radius;

public:
    Light(fixedVector position, double radius);
    double getIntensity(fixedVector sensor, double norm);
    void draw(matplot::axes_handle ax);
};