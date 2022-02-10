#include <iostream>
#include <opencv2/core.hpp>
#include <vector>
#include "space.h"

#include <cmath>
#include <thread>
#include "gnuplot-iostream.h"
#include <boost/tuple/tuple.hpp>
#include <string>
#include <string_view>

using std::vector;
typedef vector<boost::tuple<double, double, double, double>> connection;

class Plot{
    public:
        Gnuplot gp;

        Plot& xlabel(std::string xlabel){
            gp << "set xlabel '"<<xlabel<<"'\n";
            return *this;
        }
        Plot& ylabel(std::string ylabel){
            gp << "set xlabel '"<<ylabel<<"'\n";
            return *this;

        }
        Plot& setTitle(std::string title){
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

        void drawGraph(std::vector<boost::tuple<double, double>>& v, 
                            std::vector<boost::tuple<double, double, double, double>>& conn,
                            std::vector<boost::tuple<double, double, double>>& obs
        ){
            gp << "plot '-' using 1:2 with points pt 7, '-' using 1:2:3:4 with vector notitle, '-' using 1:2:3 with circles notitle\n";
            gp.send1d(v);
            gp.send1d(conn);
            gp.send1d(obs);
        }

        void drawObstacle(std::vector<boost::tuple<double, double, double>>& v){
            gp << "plot '-' using 1:2:3 with circles notitle\n";
            gp.send1d(v);
        }

        void drawSphere(double x,double y,double r){
            // For 3D Spherical Obstacle
            gp << "x = "<<x<<"\n";
            gp << "y = "<<y<<"\n";
            gp << "r = "<<r<<"\n";
            gp << "set urange [0:2*pi]\n";
            gp << "set vrange [-pi/2:pi/2]\n";
            gp << "fx(x,v,u) = x + r*cos(v)*cos(u)\n";
            gp << "fy(y,v,u) = y + r*cos(v)*sin(u)\n";
            gp << "fz(z,v)   = z + r*sin(v)\n";
            gp << "splot fx(v,u),fy(v,u),fz(v)\n";
        }

};

void formGraph(connection& connectionVector,vector<boost::tuple<double, double>>& nodeVector,Node& node){
    nodeVector.push_back(boost::make_tuple(node.x, node.y));
    if(!node.childNodes.size()>0){
        return;
    }
    for(Node* childnode : node.childNodes){
        // Add to connection
        connectionVector.push_back(boost::make_tuple(node.x, node.y, childnode->x-node.x, childnode->y-node.y));
        formGraph(connectionVector, nodeVector, *childnode);
    }
    
}

int main(){

    Space space;
    space.init();
    bool found=false;
    Plot plot;
    plot.setTitle("RRT").xlabel("X").ylabel("Y");

    vector<boost::tuple<double, double, double, double>> connectionVector;
    vector<boost::tuple<double, double, double>> obstacleVector;
    vector<boost::tuple<double, double>> nodeVector;
    
    vector<double> xo;
    vector<double> yo;
    for(Obstacle obstacle: space.obstacles ){
        obstacleVector.push_back(boost::make_tuple(obstacle.x,obstacle.y,obstacle.r));
    }

    while (!found)
    {   
        found = space.solve();

        int c=0;
        int a;

        nodeVector.clear();
        connectionVector.clear();
        Node& currentNode = space.start;
        
        formGraph(connectionVector, nodeVector, currentNode);

        //plot.drawObstacle(obstacleVector);
        plot.drawGraph(nodeVector, connectionVector, obstacleVector);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }
    
    return 0;
} 