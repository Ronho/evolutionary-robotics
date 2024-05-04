#include <vector>
#include <stdexcept>

#include "types.h"

class LightController {
public:
    /**
     * @brief Calculate the speed per wheel of the vehicle.
     * 
     * @param sensorValues Measure values of the light sensor. Expects two
     * values - at index 0 the value for the left sensor, at 1 for the right
     * sensor.
     * @return Speed per wheel - at index 0 for the left wheel and at 1 for the
     * right wheel. 
     */
    virtual fixedVector control(std::vector<double> sensorValues) = 0;
};

class BraitenbergAgressor : public LightController {
public:
    /**
     * @brief Calculate the speed per wheel of the vehicle.
     * 
     * @param sensorValues Measure values of the light sensor. Expects two
     * values - at index 0 the value for the left sensor, at 1 for the right
     * sensor.
     * @return Speed per wheel - at index 0 for the left wheel and at 1 for the
     * right wheel. 
     */
     fixedVector control(std::vector<double> sensorValues);
};

class BraitenbergFear : public LightController {
public:
    /**
     * @brief Calculate the speed per wheel of the vehicle.
     * 
     * @param sensorValues Measure values of the light sensors. Expects two
     * values - at index 0 the value for the left sensor, at 1 for the right
     * sensor.
     * @return Speed per wheel - at index 0 for the left wheel and at 1 for the
     * right wheel. 
     */
     fixedVector control(std::vector<double> sensorValues);
};

/**
 * @brief Construct a LightController based on the choosen strategy.
 * 
 * @param identifier Choosen strategy.
 * @return The LightController itself.
 */
LightController* buildLightController(std::string identifier);

class ProximityController {
public:
     /**
      * @brief Calculate the speed per wheel of the vehicle.
      * 
      * @param sensorValues Measured values of the proximity sensors. Expects
      * three values where 0 is the left, 1 the middle, and 2 the right sensor.
      * @return Speed per wheel - at index 0 for the left wheel and at 1 for the
      * right wheel. 
      */
     virtual fixedVector control(std::vector<double> sensorValues) = 0;
};

class HandMadeProxCon : public ProximityController {
public:
     /**
      * @brief Calculate the speed per wheel of the vehicle.
      * 
      * @param sensorValues Measured values of the proximity sensors. Expects
      * three values where 0 is the left, 1 the middle, and 2 the right sensor.
      * @return Speed per wheel - at index 0 for the left wheel and at 1 for the
      * right wheel. 
      */
     fixedVector control(std::vector<double> sensorValues);
};