#include <iostream>
#include <opencv2/core.hpp>
#include <vector>
#include "space.h"

#include <cmath>
#include <matplot/matplot.h>


using namespace matplot;
using std::vector;

int main(){

    Space space;
    space.init();
    bool found=false;
    while (!found)
    {   
        found = space.solve();
        for(Obstacle obstacle: space.obstacles ){
            vector<double> x((double) obstacle.x);
            vector<double> y((double) obstacle.y);

            scatter(x, y);
            show();

        }

        int a;
        std::cin>>a;
        std::cout<<"Solution found: "<<found;
    }
    
    // space.start
    // space.goal
    // space.obstacles
    // space.nodes

    // using namespace matplot;
    // auto t = iota(0, pi / 500, 40 * pi);
    // auto xt =
    //     transform(t, [](auto t) { return (3. + cos(sqrt(32.) * t)) * cos(t); });
    // auto yt = transform(t, [](auto t) { return sin(sqrt(32.) * t); });
    // auto zt =
    //     transform(t, [](auto t) { return (3. + cos(sqrt(32.) * t)) * sin(t); });
    // plot3(xt, yt, zt);
    // axis(equal);
    // xlabel("x(t)");
    // ylabel("y(t)");
    // zlabel("z(t)");

    // auto ax = gca();

    // show();
    
    return 0;
} 