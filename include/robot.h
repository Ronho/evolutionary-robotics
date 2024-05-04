#include <matplot/matplot.h>
#include <vector>

#include "types.h"

class Robot
{
public:
    fixedVector position;
    double heading;
    double radius;
    std::vector<double> sensors;

private:
    /**
     * @brief Get the position of a sensor relative to the robot's center.
     *
     * If the index is out of bounds, throws an error.
     *
     * @param idx Index of the sensor.
     *
     * @return Relative position of the sensor.
     */
    fixedVector getRelativeSensorVector(int idx);

public:
    /**
     * @brief Construct a new Robot object.
     *
     * A robot's body is represented by a circle. The wheels are on the left and
     * right perpendicular to the robots heading.
     *
     * @param position Initial position of the robot.
     * @param heading Initial direction, the vector is heading.
     * @param radius Radius of the robot's body. Depending on the driving
     * strategy, this may influence the robot's behaviour.
     * @param sensors Angles of the sensors relative to the heading angle. E.g.,
     * 0 indicates that the sensor points in the same direction as the heading
     * vector while 45 means that the sensor points 45° to the right of the
     * heading vector.
     */
    Robot(
        fixedVector position,
        double heading,
        double radius,
        std::vector<double> sensors
    );

    /**
     * @brief Calculate the new position and heading angle after driving.
     * 
     * TODO: Explain different ways.
     * 
     * @param speed Speed that the vehicle has in distance per time unit.
     * @param timeSteps Number of time units to calculate into the future. Due
     * to numerical inaccuracy, using larger values can result in different
     * outcomes compared to multiple smaller steps.
     */
    void drive(fixedVector speed, double timeSteps);

    /**
     * @brief Get the relative heading vector with the size of radius.
     * 
     * @return Heading vector relative to the robots center (so starting from
     * [0, 0]).
     */
    fixedVector getHeadingVector();

    /**
     * @brief Get the exact position of a sensor in Euclidean space.
     * 
     * @param idx Index of the vector. Throws an error if the index is out of
     * bounds.
     * 
     * @return Position of the sensor. 
     */
    fixedVector getSensorPosition(int idx);

    /**
     * @brief Draw the robot onto a 2D canvas.
     * 
     * @param ax Canvas to draw into.
     * @param only_trajectory Whether to draw only the heading vector (true) or
     * the body and the sensors as well (false). The heading vector is displayed
     * in red.
     */
    void draw(matplot::axes_handle ax, bool only_trajectory = true);
};