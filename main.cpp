#include <iostream>
#include <opencv2/core.hpp>
#include <vector>
#include "space.h"

#include <cmath>
#include <matplot/matplot.h>
#include <thread>
#include "gnuplot-iostream.h"
#include <boost/tuple/tuple.hpp>
#include <string>

class Plot{
    public:
        Gnuplot gp;

        Plot& xlabel(std::string_view xlabel){
            gp << "set xlabel '"<<xlabel<<"'\n";
            return *this;
        }
        Plot& ylabel(std::string_view ylabel){
            gp << "set xlabel '"<<ylabel<<"'\n";
            return *this;

        }
        Plot& setTitle(std::string_view title){
            gp << "set title '"<<title<<"'\n";
            return *this;
        }
        Plot& setGrid(bool grid){
            if(grid){
                gp<<"set grid\n";
            }else{
                gp<<"unset grid\n";
            }
            return *this;
        }

        void resetSession(){
            gp << "reset session";
        }

        void linePlot(std::vector<double>& v){
            gp << "plot '-' with lines title 'v0'\n";
            gp.send(v);
        }

        void drawVector(std::vector<boost::tuple<double, double, double, double>>& v){
            gp << "plot '-' using 1:2:3:4 with vector \n";//<<std::endl;
            gp.send1d(v);
        }

};

using namespace matplot;
using std::vector;

int main(){

    Space space;
    space.init();
    space.addNode();
    space.addNode();
    space.solve();
    bool found=false;
    Plot plot;
    // vector<double> v;
    vector<boost::tuple<double, double, double, double>> v;
    
    //v.push_back(boost::make_tuple(1,1,3,4));
    v.push_back(boost::make_tuple(1,1,2,3));
    
    plot.setTitle("lines").xlabel("x").ylabel("y").drawVector(v);

    // vector<double> xo;
    // vector<double> yo;
    // for(Obstacle obstacle: space.obstacles ){
    //     xo.push_back((double) obstacle.x);
    //     yo.push_back((double) obstacle.y);

    // }
    // auto o = scatter(xo, yo);

	// Create a script which can be manually fed into gnuplot later:
	//    Gnuplot gp(">script.gp");
	// Create script and also feed to gnuplot:
	//    Gnuplot gp("tee plot.gp | gnuplot -persist");
	// Or choose any of those options at runtime by setting the GNUPLOT_IOSTREAM_CMD
	// environment variable.

	// Gnuplot vectors (i.e. arrows) require four columns: (x,y,dx,dy)
	// std::vector<boost::tuple<double, double, double, double> > pts_A;

	// // You can also use a separate container for each column, like so:
	// std::vector<double> pts_B_x;
	// std::vector<double> pts_B_y;
	// std::vector<double> pts_B_dx;
	// std::vector<double> pts_B_dy;

	// You could also use:
	//   std::vector<std::vector<double> >
	//   boost::tuple of four std::vector's
	//   std::vector of std::tuple (if you have C++11)
	//   arma::mat (with the Armadillo library)
	//   blitz::Array<blitz::TinyVector<double, 4>, 1> (with the Blitz++ library)
	// ... or anything of that sort

	// for(double alpha=0; alpha<1; alpha+=1.0/24.0) {
	// 	double theta = alpha*2.0*3.14159;
	// 	pts_A.push_back(boost::make_tuple(
	// 		 cos(theta),
	// 		 sin(theta),
	// 		-cos(theta)*0.1,
	// 		-sin(theta)*0.1
	// 	));

	// 	pts_B_x .push_back( cos(theta)*0.8);
	// 	pts_B_y .push_back( sin(theta)*0.8);
	// 	pts_B_dx.push_back( sin(theta)*0.1);
	// 	pts_B_dy.push_back(-cos(theta)*0.1);
	// }

	// Don't forget to put "\n" at the end of each line!
	//gp << "set xrange [-2:2]\nset yrange [-2:2]\n";
	// '-' means read from stdin.  The send1d() function sends data to gnuplot's stdin.
	//gp << "plot '-' with vectors title 'pts_A', '-' with vectors title 'pts_B'\n";
    // gp << "set parametric";
    // gp << "set urange [0:3.0/2*pi]";
    // gp << "set vrange [-pi/2:pi/2]";
    // gp << "fx(v,u) = r*cos(v)*cos(u)";
    // gp << "fy(v,u) = r*cos(v)*sin(u)";
    // gp << "fz(v)   = r*sin(v)";
    // gp << "splot fx(v,u),fy(v,u),fz(v)";
	// gp.send1d(pts_A);
	// gp.send1d(boost::make_tuple(pts_B_x, pts_B_y, pts_B_dx, pts_B_dy));




    // while (!found)
    // {   
    //     found = space.solve();


    //     int c=0;
    //     int a;

    //     // hold(on);
    //     scatter(xo, yo);

    //     vector<double> xx;
    //     vector<double> yy;
    //     vector<vector<double>> lx;
    //     vector<vector<double>> ly;

    //     for(Node node: space.nodes ){
    //         std::cout<<c<<" "<<node.x<<" "<<node.y<<std::endl;
    //         xx.push_back((double) node.x);
    //         yy.push_back((double) node.y);

    //         lx[c].push_back((double) node.x);
    //         ly[c].push_back((double) node.y);

    //         lx[c].push_back((double) node.childNode->x);
    //         ly[c].push_back((double) node.childNode->y);
    //         c++;
    //     }
    //                 //plot(lx,ly, "k");
    // plot(vector_2d{{1, 4}, {2, 5}, {3, 6}});
    // show();

    //     // auto l = scatter(xx, yy);
    //     // l->marker_face_color({0.f, .7f, .7f});
    //     // hold(off);

    //     std::this_thread::sleep_for(std::chrono::milliseconds(200));

    //     std::cin>>a;

    //     std::cout<<"Solution found: "<<found;
    // }
    
    // // space.start
    // // space.goal
    // // space.obstacles
    // // space.nodes

    // // using namespace matplot;
    // // auto t = iota(0, pi / 500, 40 * pi);
    // // auto xt =
    // //     transform(t, [](auto t) { return (3. + cos(sqrt(32.) * t)) * cos(t); });
    // // auto yt = transform(t, [](auto t) { return sin(sqrt(32.) * t); });
    // // auto zt =
    // //     transform(t, [](auto t) { return (3. + cos(sqrt(32.) * t)) * sin(t); });
    // // plot3(xt, yt, zt);
    // // axis(equal);
    // // xlabel("x(t)");
    // // ylabel("y(t)");
    // // zlabel("z(t)");

    // // auto ax = gca();

    // // show();
    
    return 0;
} 