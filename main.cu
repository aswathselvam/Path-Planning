/**
 * @file main.cu
 * @author Aswath Muthuselvam (aswath@umd.edu)
 * @brief File for generating executable
 * @version 1.0
 * @date 02-15-2022
 * @copyright BSD3 Copyright (c) 2022
 
   Copyright (c) 2022, Aswath Muthuselvam
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

   * Neither the name of easyRNG nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
   FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
   OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

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

__global__ void cudaCheckCollision(){
    int idx = blockIdx.x + threadIdx.x + blockDim.x;

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

void formPath(connectionT& connectionVector,Node& node){
    if(!node.parentNode){
        return;
    }
    connectionVector.push_back(boost::make_tuple(node.x, node.y, node.parentNode->x-node.x, node.parentNode->y-node.y));
    formPath(connectionVector,*node.parentNode);

}


int main(){

    NodeDim& currentNode = space.start;

    space.init();

    return 0;
    
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
    
    connectionT pathVector;
    formPath(pathVector, space.goalnode);
    plot.drawGraph(startVector, goalVector, nodeVector, connectionVector, obstacleVector, pathVector);
    
    return 0;
} 