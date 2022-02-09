#include "space.h"
#include <stdlib.h>    
#include <cmath>
#include <iostream>

void Space::init(){
    srand (1);
    start.x = 10;
    start.y = 10;

    goal.x = 50;
    goal.y = 50;

    for(int i = 0; i<5 ;i++){
        Obstacle obstacle;
        obstacle.x=(5-i)*10;
        obstacle.y=i*10;
        obstacle.r=i*5;
        obstacles.push_back(obstacle);
    }
}

Node& Space::addNode(){
    Node* node = new Node{rand() % 50 + 1, rand() % 50 + 1};
    double distt=9999;
    Node& nearestnode = getNearestNode(distt,this->start,*node);
    addConnection(nearestnode, *node);
    std::cout<<"Nearest node.childnode: "<<nearestnode.childNodes[0] << " , size: "<<nearestnode.childNodes.size()<<std::endl;

    double dist=3;
    double mag = sqrt(pow(nearestnode.x-node->x,2) + pow(nearestnode.y-node->y,2) );
    node->x = dist*(node->x - nearestnode.x)/sqrt(mag); 
    node->y = dist*(node->y - nearestnode.y)/sqrt(mag); 

    // if(checkCollision(*node)){
    //     delete node;
    //     return addNode();
    // }
    return *node;

}

bool Space::checkCollision(Node& node){
    for(Obstacle& obstacle : this->obstacles){
        if(sqrt(pow(obstacle.x-node.x,2) + pow(obstacle.y-node.y,2) ) < obstacle.r ){
            return true;
        }
    }
    return false;
}

void Space::addConnection(Node& a, Node& b){
    a.childNodes.push_back(&b);
}

Node& Space::getNearestNode(double& min_dist, Node& currentNode, Node& node){
    Node* nearestnode = nullptr;
    if(currentNode.childNodes.size()<1){
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

bool Space::solve(){
    Node& node = addNode();

    //std::cout<<"SOLVE() node.childnode: "<<node.childNodes[0]<<std::endl;

    if(sqrt(pow(this->goal.x-node.x,2) + pow(this->goal.y-node.y,2) ) < 5 ){
        return true;
    }
    return false;
}