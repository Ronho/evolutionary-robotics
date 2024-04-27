#include <stdexcept>

#include "types.h"

class Map {
public:
    fixedVector xlim;
    fixedVector ylim;

public:
    virtual fixedVector clip(fixedVector position) = 0;
};

class Torus : public Map {
public:
    fixedVector xlim;
    fixedVector ylim;

public:
    Torus(fixedVector xlim, fixedVector ylim);
    fixedVector clip(fixedVector position);
};

class Bounded : public Map {
public:
    fixedVector xlim;
    fixedVector ylim;

public:
    Bounded(fixedVector xlim, fixedVector ylim);
    fixedVector clip(fixedVector position);
};

Map* buildMap(std::string identifier);