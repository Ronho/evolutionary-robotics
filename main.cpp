#include <iostream>
#include <matplot/matplot.h>
#include <set>
#include "config.h"

class Robot {
public:
    // Where the robot is located.
    std::array<double, 2> position;
    // Where the robot is heading.
    double angle;
    // Radius (just for visualization).
    double radius;
    // Speed of the left (idx 0) and right (idx 1) wheel.
    std::array<double, 2> speed;
};

int main()
{
    std::cout << "You are running version " << EvoRob_VERSION_MAJOR << "." << EvoRob_VERSION_MINOR << "." << std::endl;
    try
    {
        Robot rob;
        rob.position = {0.5, 0.5};
        rob.angle = 90;
        rob.radius = 0.1;
        // {0, 1};
        double arrowLength = 0.3;
        matplot::arrow(rob.position[0], rob.position[1], rob.position[0] + 0*arrowLength, rob.position[1] + arrowLength); // x1, y1, x2, y2
        matplot::arrow(rob.position[0], rob.position[1], rob.position[0] + arrowLength, rob.position[1] + arrowLength); // x1, y1, x2, y2
        matplot::arrow(rob.position[0], rob.position[1], rob.position[0] + (arrowLength*(22.5/90)), rob.position[1] + arrowLength); // x1, y1, x2, y2
        matplot::arrow(rob.position[0], rob.position[1], rob.position[0] + (arrowLength*(22.5/90)), rob.position[1] - arrowLength); // x1, y1, x2, y2
        matplot::ellipse(rob.position[0]-(rob.radius / 2), rob.position[1]-(rob.radius / 2), rob.radius, rob.radius); // x, y, w, h
        matplot::xlim({0, 1});
        matplot::ylim({0, 1});
        matplot::show();
    }
    catch (const std::runtime_error &e)
    {
        // If you encounter the "popen() failed!" error, you likely have not
        // installed gnuplot correctly.
        std::cerr << "Runtime error: " << e.what() << std::endl;
    }
    return 0;
}
