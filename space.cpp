#include "space.h"
#include <stdlib.h>    
#include <cmath>
#include <iostream>
#include <limits>
#include <typeinfo>
#include <variant>


template class Space<Node3D, Obstacle3D>;

Obstacle::Obstacle(int i){
    this->x=(5-i)*10;
    this->y=i*10;
    this->r=i*1;
}

Obstacle3D::Obstacle3D(int i){
    this->x=(5-i)*10;
    this->y=i*10;
    this->z=i;
    this->r=i*1;
}

void Node::setup(double x, double y, double){
    this->x = x;
    this->y = y;
}

void Node3D::setup(double x, double y, double z){
    this->x = x;
    this->y = y;
    this->z = z;
}

template <class NodeDim, class ObstacleDim>
void Space<NodeDim, ObstacleDim>::init(){
    start.setup(10.0, 10.0, 10.0);
    goal.setup(10.0, 50.0, 50.0);

    srand (1);

    for(int i = 0; i<5 ;i++){
        ObstacleDim obstacle(i);
        obstacles.push_back(obstacle);
    }
}

template <class NodeDim, class ObstacleDim>
NodeDim& Space<NodeDim, ObstacleDim>::addNode(){
    NodeDim* node = new NodeDim{rand() % 100 + 1, rand() % 100 + 1, rand() % 100 + 1};

    double inf=std::numeric_limits<double>::max();
    NodeDim& nearestnode = getNearestNode(inf,this->start,*node);

    directionComponent(nearestnode, *node);

    if(checkCollision(*node)){
        delete node;
        return addNode();
    }
    addConnection(nearestnode, *node);
    return *node;
}

template <class NodeDim, class ObstacleDim>
double Space<NodeDim, ObstacleDim>::L2(Obstacle& obstacle, Node& node){
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
void Space<NodeDim, ObstacleDim>::addConnection(NodeDim& a, NodeDim& b){
    a.childNodes.push_back(&b);
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
            return true;
        }
    }
    return false;
}