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


bool DIM3 = true;
// #ifdef DIM3{
    // typedef vector<boost::tuple<double, double, double, double, double, double>> connectionT;
    // typedef vector<boost::tuple<double, double, double>> nodeT;
    // typedef vector<boost::tuple<double, double, double, double>> obstacleT;
// }else{
    typedef Node NodeDim;
    typedef Obstacle ObstacleDim;
    typedef vector<boost::tuple<double, double, double, double>> connectionT;
    typedef vector<boost::tuple<double, double>> nodeT;
    typedef vector<boost::tuple<double, double, double>> obstacleT;
// }

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

        void drawGraph(std::vector<boost::tuple<double, double>>& start,
                            std::vector<boost::tuple<double, double>>& goal,
                            std::vector<boost::tuple<double, double>>& nodes, 
                            std::vector<boost::tuple<double, double, double, double>>& connections,
                            std::vector<boost::tuple<double, double, double>>& obstacles
        ){
            gp << "plot '-' using 1:2 with points pt 35 ps 3 title \"Start\", \
            '-' using 1:2 with points pt 35 ps 3 title \"goal\", \
             '-' using 1:2 with points pt 7 title \"Nodes\", \
             '-' using 1:2:3:4 with vector title \"Connections\", \
             '-' using 1:2:3 with circles fillstyle pattern 4 transparent lc rgb '#990000' title \"Obstacles\"\n";

            gp.send1d(start);
            gp.send1d(goal);
            gp.send1d(nodes);
            gp.send1d(connections);
            gp.send1d(obstacles);
        }

        void drawGraph3D(nodeT& start,
                            nodeT& goal,
                            nodeT& nodes, 
                            connectionT& connections,
                           obstacleT& obstacles
        ){
            gp << "splot '-' using 1:2:3 with points pt 35 ps 3 title \"Start\", \
            '-' using 1:2:3 with points pt 35 ps 3 title \"goal\", \
            '-' using 1:2:3 with points pt 7 title \"Nodes\", \
            '-' using 1:2:3:4:5:6 with vector title \"Connections\", \
            '-' using 1:2:3:4 with circles fillstyle pattern 4 transparent lc rgb '#990000' title \"Obstacles\"\n";

            gp.send1d(start);
            gp.send1d(goal);
            gp.send1d(nodes);
            gp.send1d(connections);
            gp.send1d(obstacles);
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

void formGraph(connectionT& connectionVector,nodeT& nodeVector,Node& node){
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

    Space<NodeDim, ObstacleDim> space;
    NodeDim& currentNode = space.start;

    space.init();
    bool found=false;
    Plot plot;
    // plot.setTitle("RRT").xlabel("X").ylabel("Y");

    connectionT connectionVector;
    obstacleT obstacleVector;
    nodeT nodeVector;
    nodeT startVector;
    nodeT goalVector;
    
    for(Obstacle obstacle: space.obstacles ){
        obstacleVector.push_back(boost::make_tuple(obstacle.x,obstacle.y,obstacle.r));
    }

    startVector.push_back(boost::make_tuple(space.start.x, space.start.y));
    goalVector.push_back(boost::make_tuple(space.goal.x, space.goal.y));

    while (!found)
    {   
        found = space.solve();

        int c=0;
        int a;

        nodeVector.clear();
        connectionVector.clear();
        
        formGraph(connectionVector, nodeVector, currentNode);
        plot.drawGraph(startVector, goalVector, nodeVector, connectionVector, obstacleVector);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }
    
    return 0;
} 