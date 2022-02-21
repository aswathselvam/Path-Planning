/**
 * @file space.cpp
 * @author Aswath Muthuselvam (aswath@umd.edu)
 * @brief RRT Logic implementation file.
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

#include "space.h"
#include <stdlib.h>    
#include <cmath>
#include <iostream>
#include <limits>
#include <typeinfo>
#include <variant>

using std::cout;
using std::endl; 

template class Space<Node, Obstacle>;

Obstacle::Obstacle(){
    this->x=(5)*10;
    this->y=1*10;
    this->r=1*1;
}

Obstacle::Obstacle(int i){
    this->x=(5-i)*10;
    this->y=i*10;
    this->r=i*1;
}

Obstacle3D::Obstacle3D(int i){
    this->x=(5-i)*11;
    this->y=i*11;
    this->z=i*2;
    this->r=i*3;
}

void Node::setup(double x, double y, double z){
    this->x = x;
    this->y = y;
}

void Node3D::setup(double x, double y, double z){
    this->x = x;
    this->y = y;
    this->z = z;
}

template <class NodeDim, class ObstacleDim>
Space<NodeDim, ObstacleDim>::Space() : goalnode(goal){

}

template <class NodeDim, class ObstacleDim>
void Space<NodeDim, ObstacleDim>::init(){
    width=300;
    height=width;
    start.setup(10.0, 10.0, 10.0);
    goal.setup(width-10.2, 50.0, 50.0);

    srand (1);

    N_OF_OBSTACLES=1000; 
    h_obstacles = new ObstacleDim[N_OF_OBSTACLES];
    for(int i = 0; i<N_OF_OBSTACLES ;i++){
        ObstacleDim obstacle(i);
        obstacle.intersected = false;
        obstacle.x=rand() % width;
        obstacle.y=rand() % height;
        obstacle.r=rand() % 3;
        h_obstacles[i] = obstacle;
        this->obstacles.push_back(obstacle);
    }

    cudaMalloc(&d_obstacles, N_OF_OBSTACLES*sizeof(ObstacleDim));
	cudaMemcpy(d_obstacles, h_obstacles, N_OF_OBSTACLES*sizeof(ObstacleDim), cudaMemcpyHostToDevice);
    
}

template <class NodeDim, class ObstacleDim>
NodeDim& Space<NodeDim, ObstacleDim>::addNode(){

    NodeDim* node = new NodeDim{rand() % width + 1.0, rand() % height + 1.0};

    double inf=std::numeric_limits<double>::max();
    NodeDim& nearestnode = getNearestNode(inf,this->start,*node);

    directionComponent(nearestnode, *node);

    bool collision = false;
    bool CUDA=false;
    if(CUDA){
        cudaCheckCollision<<<1, N_OF_OBSTACLES>>>(this, d_obstacles, *node); 
        cudaMemcpy(h_obstacles, d_obstacles, N_OF_OBSTACLES*sizeof(ObstacleDim), cudaMemcpyDeviceToHost);
        for(int i =0; i< N_OF_OBSTACLES; i++){
            ObstacleDim d = h_obstacles[i];
            if(d.intersected){
                collision=true;
                // cout<<"Collision detected at x: "<<d.x<<" y: "<<d.y<<endl;
            }
        }
    }else{
        collision = checkCollision(*node);
    }
      
    if(collision){
        delete node;
        return addNode();
    }
    addConnection(nearestnode, *node);
    return *node;
}

template <class NodeDim, class ObstacleDim>
__host__ __device__ double Space<NodeDim, ObstacleDim>::L2(Obstacle& obstacle, Node& node){
    return sqrt(pow(obstacle.x-node.x,2) + pow(obstacle.y-node.y,2) );
}

template <class NodeDim, class ObstacleDim>
double Space<NodeDim, ObstacleDim>::L2(Obstacle3D& obstacle, Node3D& node){
    return sqrt(pow(obstacle.x-node.x,2) + pow(obstacle.y-node.y,2) + pow(obstacle.z-node.z,2) );
}

template <class NodeDim, class ObstacleDim>
double Space<NodeDim, ObstacleDim>::L2(Node& n1, Node& n2){
    return sqrt(pow(n1.x-n2.x,2) + pow(n1.y-n2.y,2) );
}

template <class NodeDim, class ObstacleDim>
double Space<NodeDim, ObstacleDim>::L2(Node3D& n1, Node3D& n2){
    return sqrt(pow(n1.x-n2.x,2) + pow(n1.y-n2.y,2) + pow(n1.z-n2.z,2) );
}


template <class NodeDim, class ObstacleDim>
void Space<NodeDim, ObstacleDim>::directionComponent(Node& n1to, Node& n2){
    double mag = L2(n1to, n2);
    double dist= mag > 3 ? 3 : mag;
    n2.x = n1to.x + dist*(n2.x - n1to.x)/mag; 
    n2.y = n1to.y + dist*(n2.y - n1to.y)/mag;
}

template <class NodeDim, class ObstacleDim>
void Space<NodeDim, ObstacleDim>::directionComponent(Node3D& n1to, Node3D& n2){
    double mag = L2(n1to, n2);
    double dist= mag > 3 ? 3 : mag;
    n2.x = n1to.x + dist*(n2.x - n1to.x)/mag; 
    n2.y = n1to.y + dist*(n2.y - n1to.y)/mag;
    n2.z = n1to.z + dist*(n2.z - n1to.z)/mag;
}

template <class NodeDim, class ObstacleDim>
bool Space<NodeDim, ObstacleDim>::checkCollision(NodeDim& node){
    for(ObstacleDim& obstacle : this->obstacles){
        if(L2(obstacle, node) < 2*obstacle.r){
            return true;
        }
    }
    return false;
}

template <class NodeDim, class ObstacleDim>
__global__ void cudaCheckCollision(Space<NodeDim, ObstacleDim>* spacep, Obstacle* d_obstacles, Node node){
    int idx = blockIdx.x + threadIdx.x ;//+ blockDim.x;
    double dist = spacep->L2(d_obstacles[idx], node);
    // printf("idx: %d, dist: %f, radius: %f\n",idx, dist, d_obstacles[idx].r);
    d_obstacles[idx].intersected=false;
    if(dist<2*d_obstacles[idx].r){
        d_obstacles[idx].intersected=true;
    }
}

template <class NodeDim, class ObstacleDim>
void Space<NodeDim, ObstacleDim>::addConnection(NodeDim& a, NodeDim& b){
    a.childNodes.push_back(&b);
    b.parentNode=&a;
}

template <class NodeDim, class ObstacleDim>
NodeDim& Space<NodeDim, ObstacleDim>::getNearestNode(double& min_dist, NodeDim& currentNode, NodeDim& node){
    NodeDim* nearestnode = nullptr;
    if(&currentNode == &start && currentNode.childNodes.size()<1){
        return currentNode;
    }
    for(NodeDim* childNode: currentNode.childNodes){
        float dist = L2(*childNode, node);
        if(dist < min_dist){
            nearestnode = childNode;
            min_dist = dist;
        }
        NodeDim& nearestnodeCandidate = getNearestNode(min_dist, *childNode, node);
        if(&nearestnodeCandidate!=nullptr){
                nearestnode = &nearestnodeCandidate;
        }
    }

    return *nearestnode;
}

template <class NodeDim, class ObstacleDim>
bool Space<NodeDim, ObstacleDim>::solve(){
    NodeDim* node = &addNode();

    if(node!=nullptr){
        if(L2(goal,*node) < 5){
            goalnode = *node;
            return true;
        }
    }
    return false;
}