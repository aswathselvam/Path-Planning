#include <iostream>
#include <opencv2/core.hpp>
#include <vector>
#include "space.h"
// #include <cuda_runtime.h> 

#include <cmath>
#include <thread>
#include "gnuplot-iostream.h"
#include <boost/tuple/tuple.hpp>
#include <string>
#include <string_view>
#include <typeinfo>


using std::vector;

typedef Node NodeDim;
typedef Obstacle ObstacleDim;
typedef vector<boost::tuple<double, double, double, double>> connectionT;
typedef vector<boost::tuple<double, double>> nodeT;
typedef vector<boost::tuple<double, double, double>> obstacleT;
#include "plot.h"

Space<NodeDim, ObstacleDim> space;

__global__ void cuda_solve(){
    printf("\nKernel: Block ID: %d",blockIdx.x);
    printf("\nKernel: Block dim: %d",blockDim.x);
    printf("\nKernel: Thread ID: %d",threadIdx.x);
}

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

    NodeDim& currentNode = space.start;
    cuda_solve<<<10,10>>>(); 

    space.init();
    bool found=false;
    Plot plot;
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

        nodeVector.clear();
        connectionVector.clear();
        
        formGraph(connectionVector, nodeVector, currentNode);
        plot.drawGraph(startVector, goalVector, nodeVector, connectionVector, obstacleVector);

        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    }
    
    return 0;
} 