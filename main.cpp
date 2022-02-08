#include <iostream>
#include <opencv2/core.hpp>
#include <vector>
#include "space.h"

#include <cmath>
#include <matplot/matplot.h>
#include <thread>



using namespace matplot;
using std::vector;

int main(){

    Space space;
    space.init();
    space.addNode();
    space.addNode();
    space.solve();
    bool found=false;

    vector<double> xo;
    vector<double> yo;
    for(Obstacle obstacle: space.obstacles ){
        xo.push_back((double) obstacle.x);
        yo.push_back((double) obstacle.y);

    }
    auto o = scatter(xo, yo);


    while (!found)
    {   
        found = space.solve();


        int c=0;
        int a;

        // hold(on);
        scatter(xo, yo);

        vector<double> xx;
        vector<double> yy;
        vector<vector<double>> lx;
        vector<vector<double>> ly;

        for(Node node: space.nodes ){
            std::cout<<c<<" "<<node.x<<" "<<node.y<<std::endl;
            xx.push_back((double) node.x);
            yy.push_back((double) node.y);

            lx[c].push_back((double) node.x);
            ly[c].push_back((double) node.y);

            lx[c].push_back((double) node.childNode->x);
            ly[c].push_back((double) node.childNode->y);
            c++;
        }
                    //plot(lx,ly, "k");
    plot(vector_2d{{1, 4}, {2, 5}, {3, 6}});
    show();

        // auto l = scatter(xx, yy);
        // l->marker_face_color({0.f, .7f, .7f});
        // hold(off);

        std::this_thread::sleep_for(std::chrono::milliseconds(200));

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