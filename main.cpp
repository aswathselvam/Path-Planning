/**
 * @file main.cpp
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
#include <vector>
#include "space.h"
#include <cmath>
#include <thread>
#include <string>
#include <string_view>
#include <typeinfo>
#include <chrono>

using std::vector;
using std::cout;
using std::endl;

typedef Node NodeDim;
typedef Obstacle ObstacleDim;

#ifdef ENABLE_VISUALIZATION
#include "gnuplot-iostream.h"
#include <boost/tuple/tuple.hpp>
#include <opencv2/core.hpp>

typedef vector<boost::tuple<double, double, double, double>> connectionT;
typedef vector<boost::tuple<double, double>> nodeT;
typedef vector<boost::tuple<double, double, double>> obstacleT;
#include "plot.h"
#endif

Space<NodeDim, ObstacleDim> space;

#ifndef NO_CUDA
__global__ void cudaCheckCollision(){
    int idx = blockIdx.x + threadIdx.x + blockDim.x;

}
#endif

#ifdef ENABLE_VISUALIZATION
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
#endif

int main(){

    NodeDim& currentNode = space.start;

    space.init();
    
    bool found=false;

#ifdef ENABLE_VISUALIZATION
    Plot plot(space.width, space.height);
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
#endif

    int count = 0;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    cout << "Starting RRT Solver..." << endl;

    while (!found)
    {   
        found = space.solve();

#ifdef ENABLE_VISUALIZATION
        // nodeVector.clear();
        // connectionVector.clear();
        // formGraph(connectionVector, nodeVector, currentNode);
        // plot.drawGraph(startVector, goalVector, nodeVector, connectionVector, obstacleVector);

        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
#endif

        if(count%100==0){
             std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
             cout<<"Iteration: "<<count<<"\tTime: "<<std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" <<endl;
        }
        count++;

        if (count > 50000) {
            cout << "Failed to find path after 50000 iterations." << endl;
            break;
        }
    }
    
    if (found) {
        cout << "Path found in " << count << " iterations." << endl;
        if (space.goalnode) {
             cout << "Goal reached at: " << space.goalnode->x << ", " << space.goalnode->y << endl;
        }
    }

#ifdef ENABLE_VISUALIZATION
    connectionT pathVector;
    formPath(pathVector, *space.goalnode);
    plot.drawGraph(startVector, goalVector, nodeVector, connectionVector, obstacleVector, pathVector);
#endif
    
    return 0;
}
