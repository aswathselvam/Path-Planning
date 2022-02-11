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
#include <typeinfo>

using std::vector;

typedef Node3D NodeDim;
typedef Obstacle3D ObstacleDim;
typedef vector<boost::tuple<double, double, double, double, double, double>> connectionT;
typedef vector<boost::tuple<double, double, double>> nodeT;
typedef vector<boost::tuple<double, double, double, double>> obstacleT;
#include "plot.h"


void formGraph3D(connectionT& connectionVector,nodeT& nodeVector,Node3D& node3d){
    nodeVector.push_back(boost::make_tuple(node3d.x, node3d.y, node3d.z));
    if(!node3d.childNodes.size()>0){
        return;
    }
    for(Node3D* childnode : node3d.childNodes){
        // Add to connection
        connectionVector.push_back(boost::make_tuple(node3d.x, node3d.y, node3d.z, childnode->x-node3d.x, childnode->y-node3d.y, childnode->z-node3d.z));
        formGraph3D(connectionVector, nodeVector, *childnode);
    }   
}

int main(){

    Space<NodeDim, ObstacleDim> space;
    NodeDim& currentNode = space.start;

    space.init();
    bool found=false;
    Plot plot;
    connectionT connectionVector;
    obstacleT obstacleVector;
    nodeT nodeVector;
    nodeT startVector;
    nodeT goalVector;
    
    for(ObstacleDim obstacle: space.obstacles ){
        obstacleVector.push_back(boost::make_tuple(obstacle.x,obstacle.y,obstacle.z,obstacle.r));
    }
    startVector.push_back(boost::make_tuple(space.start.x, space.start.y, space.start.z));
    goalVector.push_back(boost::make_tuple(space.goal.x, space.goal.y, space.goal.z));
            plot.drawobs(obstacleVector);


    while (!found)
    {   
        found = space.solve();

        int c=0;
        int a;

        nodeVector.clear();
        connectionVector.clear();
        
        formGraph3D(connectionVector, nodeVector, currentNode);
        plot.drawGraph3D(startVector, goalVector, nodeVector, connectionVector, obstacleVector);
        // break;
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    }
    
    return 0;
} 