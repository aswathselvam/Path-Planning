#include "space.h"
#include <stdlib.h>    
#include <cmath>

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

Node Space::addNode(){
    Node node = {rand() % 50 + 1, rand() % 50 + 1, nullptr, nullptr};
    // node.x = 
    // node.y = rand() % 50 + 1;

    Node nearestnode = getNearestNode(node);
    double dist=3;
    double mag = sqrt(pow(nearestnode.x-node.x,2) + pow(nearestnode.y-node.y,2) );
    node.x = dist*(node.x - nearestnode.x)/sqrt(mag); 
    node.y = dist*(node.y - nearestnode.y)/sqrt(mag); 

    if(checkCollision(node)){
        return addNode();
    }

    return node;
}

bool Space::checkCollision(Node node){
    for(Obstacle obstacle : this->obstacles){
        if(sqrt(pow(obstacle.x-node.x,2) + pow(obstacle.y-node.y,2) ) < obstacle.r ){
            return true;
        }
    }
    return false;
}

void Space::addConnection(Node a, Node b){
    a.childNode = &b;
}

Node Space::getNearestNode(Node node){
    Node nearestnode;
    float min_dist=99999;
    for(Node n: this->nodes){
        float dist = sqrt(pow(n.x-node.x,2) + pow(n.x-node.x,2) );
        if(dist < min_dist){
            nearestnode = n;
            min_dist = dist;
        }
    }
    return nearestnode;
}

bool Space::solve(){
    Node node = addNode();
    Node nearest_node = getNearestNode(node);
    addConnection(nearest_node, node);
    nodes.push_back(node);

    if(sqrt(pow(this->goal.x-node.x,2) + pow(this->goal.y-node.y,2) ) < 5 ){
        return true;
    }
    return false;
}