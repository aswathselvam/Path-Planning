#include "space.h"
#include <stdlib.h>    
#include <cmath>
#include <iostream>
#include <limits>
template class Space<Node>;

template <class NodeDim>
void Space<NodeDim>::init(){
    srand (1);
    start.x = 10;
    start.y = 10;

    goal.x = 10;
    goal.y = 50;

    for(int i = 0; i<5 ;i++){
        Obstacle obstacle;
        obstacle.x=(5-i)*10;
        obstacle.y=i*10;
        obstacle.r=i*1;
        obstacles.push_back(obstacle);
    }
}

template <class NodeDim>
NodeDim& Space<NodeDim>::addNode(){
    NodeDim* node = new NodeDim{rand() % 100 + 1, rand() % 100 + 1};

    double distt=std::numeric_limits<double>::max();
    NodeDim& nearestnode = getNearestNode(distt,this->start,*node);

    double mag = sqrt(pow(nearestnode.x-node->x,2) + pow(nearestnode.y-node->y,2) );
    double dist= mag > 3 ? 3 : mag;
    node->x = nearestnode.x + dist*(node->x - nearestnode.x)/mag; 
    node->y = nearestnode.y + dist*(node->y - nearestnode.y)/mag;

    if(checkCollision(*node)){
        delete node;
        return addNode();
    }
    addConnection(nearestnode, *node);
    return *node;
}

template<class NodeDim>
double Space<NodeDim>::L2(Obstacle& obstacle, Node& node){
    return sqrt(pow(obstacle.x-node.x,2) + pow(obstacle.y-node.y,2) );
}

template<class NodeDim>
bool Space<NodeDim>::checkCollision(Node& node){
    for(Obstacle& obstacle : this->obstacles){
        if(L2(obstacle, node) < 2*obstacle.r){
            return true;
        }
    }
    return false;
}

template<class NodeDim>
void Space<NodeDim>::addConnection(NodeDim& a, NodeDim& b){
    a.childNodes.push_back(&b);
}

template<class NodeDim>
NodeDim& Space<NodeDim>::getNearestNode(double& min_dist, NodeDim& currentNode, NodeDim& node){
    Node* nearestnode = nullptr;
    if(&currentNode == &start && currentNode.childNodes.size()<1){
        return currentNode;
    }
    for(Node* childNode: currentNode.childNodes){
        float dist = sqrt(pow(childNode->x-node.x,2) + pow(childNode->y-node.y,2) );
        if(dist < min_dist){
            nearestnode = childNode;
            min_dist = dist;
        }
        Node& nearestnodeCandidate = getNearestNode(min_dist, *childNode, node);
        if(&nearestnodeCandidate!=nullptr){
                nearestnode = &nearestnodeCandidate;
        }
    }

    return *nearestnode;
}

template<class NodeDim>
bool Space<NodeDim>::solve(){
    NodeDim* node = &addNode();

    //std::cout<<"SOLVE() node.childnode: "<<node.childNodes[0]<<std::endl;
    if(node!=nullptr){
    if(sqrt(pow(this->goal.x-node->x,2) + pow(this->goal.y-node->y,2) ) < 5 ){
        return true;
    }
    }
    return false;
}