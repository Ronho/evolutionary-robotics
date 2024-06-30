#include <matplot/matplot.h>

#include "types.h"

// Assumption: 1 is lower left corner, 2 is upper right corner.

class Wall {
public:
    double x1;
    double x2;
    double y1;
    double y2;

public:
    /**
     * @brief Construct a new Wall object.
     * 
     * It is assumed that point 1 represents the lower left corner and point 2
     * the upper right corner of the wall.
     * 
     * @param x1 X of point 1.
     * @param y1 Y of point 1.
     * @param x2 X of point 2.
     * @param y2 Y of point 2.
     */
    Wall(double x1, double y1, double x2, double y2);

    /**
     * @brief Get the width of the wall.
     * 
     * @return Width.
     */
    double getWidth();

    /**
     * @brief Get the height of the wall.
     * 
     * @return Height.
     */
    double getHeight();

    bool within(int x, int y);
};

class Map {
public:
    fixedVector xlim;
    fixedVector ylim;

public:
    /**
     * @brief Bring objects outside of the map back into it.
     * 
     * Note, this can result in unexpected behaviour.
     * 
     * @param position Position of an object that may be outside the map.
     * 
     * @return New position guaranteed to be inside of the map. 
     */
    virtual fixedVector clip(fixedVector position) = 0;

    /**
     * @brief Draw map onto the canvas.
     * 
     * @param ax The canvas to draw onto.
     */
    virtual void draw(matplot::axes_handle ax) = 0;
};

class Torus : public Map {
public:
    /**
     * @brief Construct a new Torus object.
     * 
     * @param xlim Lower and upper limit for the x dimension.
     * @param ylim Lower and upper limit for the y dimension.
     */
    Torus(fixedVector xlim, fixedVector ylim);

    /**
     * @brief Bring objects outside of the map back into it.
     * 
     * Note, this can result in unexpected behaviour.
     * 
     * @param position Position of an object that may be outside the map.
     * 
     * @return New position guaranteed to be inside of the map. 
     */
    fixedVector clip(fixedVector position);

    /**
     * @brief Draw map onto the canvas.
     * 
     * @param ax The canvas to draw onto.
     */
    void draw(matplot::axes_handle ax);
};

class Bounded : public Map {
public:
    std::vector<Wall> walls;

public:
    /**
     * @brief Construct a new Bounded object.
     * 
     * @param xlim Lower and upper limit for the x dimension.
     * @param ylim Lower and upper limit for the y dimension.
     * @param walls Walls inside of the map.
     */
    Bounded(fixedVector xlim, fixedVector ylim, std::vector<Wall> walls);

    /**
     * @brief Bring objects outside of the map back into it.
     * 
     * Note, this can result in unexpected behaviour. Objects may still be
     * inside of walls. This is not prevented!
     * 
     * @param position Position of an object that may be outside the map.
     * 
     * @return New position guaranteed to be inside of the map. 
     */
    fixedVector clip(fixedVector position); // Does not accoutn for walls!

    /**
     * @brief Draw map onto the canvas.
     * 
     * @param ax The canvas to draw onto.
     */
    void draw(matplot::axes_handle ax);
};