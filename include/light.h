#include <matplot/matplot.h>
#include "types.h"

// Assumption: Light reaches every point in the field.
class Light {
public:
    fixedVector position;
    double radius;
    double norm;

public:
    /**
     * @brief Construct a new Light object.
     * 
     * Lights reach every part of the map!
     * 
     * @param position Position of the object in Euclidean space.
     * @param radius Radius of the light. Only for visual purposes!
     * @param norm A normalization factor. Used to scale values to a certain
     * range.
     */
    Light(fixedVector position, double radius, double norm);

    /**
     * @brief Calculates the intensity measured by a sensor.
     * 
     * @param sensor Position of the sensor.
     * @return Intensity measured by the sensor.
     */
    double getIntensity(fixedVector sensor);

    /**
     * @brief Draw the light onto the canvas.
     * 
     * @param ax The canvas to draw onto.
     */
    void draw(matplot::axes_handle ax);
};