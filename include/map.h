#include <matplot/matplot.h>

#include "types.h"

class Map {
public:
    fixedVector xlim;
    fixedVector ylim;

public:
    virtual fixedVector clip(fixedVector position) = 0;
    virtual void draw(matplot::axes_handle ax) = 0;
};

class Torus : public Map {
public:
    Torus(fixedVector xlim, fixedVector ylim);
    fixedVector clip(fixedVector position);
    void draw(matplot::axes_handle ax);
};

class Wall {
public:
    double x1;
    double x2;
    double y1;
    double y2;

public:
    Wall(double x1, double y1, double x2, double y2);
    double getWidth();
    double getHeight();
};

class Bounded : public Map {
public:
    std::vector<Wall> walls;
public:
    Bounded(fixedVector xlim, fixedVector ylim, std::vector<Wall> walls);
    fixedVector clip(fixedVector position);
    void draw(matplot::axes_handle ax);
};

Map* buildMap(std::string identifier);