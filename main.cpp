#include <iostream>
#include <opencv2/core.hpp>
#include <vector>
#include "space.h"

#include <cmath>
#include <thread>
#include "gnuplot-iostream.h"
#include <boost/tuple/tuple.hpp>
#include <string>

using std::vector;

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

        void drawNode(std::vector<boost::tuple<double, double>>& v){
            gp << "plot '-' using 1:2 with points pt 7\n";
            gp.send1d(v);
        }

        void drawConnection(std::vector<boost::tuple<double, double, double, double>>& v){
            gp << "plot '-' using 1:2:3:4 with vector notitle\n";
            gp.send1d(v);
        }

        void drawObstacle(double x,double y,double r){
            gp << "x = "<<x<<"\n";
            gp << "y = "<<y<<"\n";
            gp << "r = "<<r<<"\n";
            gp << "set urange [0:2*pi]\n";
            gp << "set vrange [-pi/2:pi/2]\n";
            gp << "fx(v,u) = x + r*cos(v)*cos(u)\n";
            gp << "fy(v,u) = y + r*cos(v)*sin(u)\n";
            gp << "fz(v)   = z + r*sin(v)\n";
            gp << "splot fx(v,u),fy(v,u),fz(v)\n";
        }

};

int main(){

    Space space;
    space.init();
    space.addNode();
    space.addNode();
    space.solve();
    bool found=false;
    Plot plot;

    vector<boost::tuple<double, double, double, double>> connectionVector;
    vector<boost::tuple<double, double, double>> obstacleVector;
    vector<boost::tuple<double, double>> nodeVector;
    
    vector<double> xo;
    vector<double> yo;
    for(Obstacle obstacle: space.obstacles ){
        obstacleVector.push_back(boost::make_tuple(obstacle.x,obstacle.y,obstacle.r));
    }
    plot.drawObstacle(obstacleVector);

    while (!found)
    {   
        found = space.solve();

        int c=0;
        int a;

        vector<double> xx;
        vector<double> yy;
        vector<vector<double>> lx;
        vector<vector<double>> ly;

        for(Node node: space.nodes ){
            std::cout<<c<<" "<<node.x<<" "<<node.y<<std::endl;
            //Add Node Visualization
            nodeVector.push_back(boost::make_tuple(node.x, node.y));

            // Add Connection Visualization
            connectionVector.push_back(node.x, node.y, node.childNode->x, node.childNode->y);
        }

        plot.drawNode(nodeVector);
        plot.drawConnection(connectionVector);

        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        std::cout<<"Solution found: "<<found;
    }
    
    return 0;
} 