/**
 * @file space.h
 * @author Aswath Muthuselvam (aswath@umd.edu)
 * @brief RRT Logic header file.
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

#include <vector>

using std::vector;

class Obstacle{
    public:
        Obstacle(int);
        double x;
        double y;
        double r;
};

class Obstacle3D{
    public:
        Obstacle3D(int);
        double x;
        double y;
        double z;
        double r;
};

class Node{
    public:
        void setup(double, double, double);
        double x;
        double y;
        Node* parentNode;
        vector<Node*> childNodes;

};

class Node3D{
    public:
        void setup(double, double, double);
        double x;
        double y;
        double z;
        Node3D* parentNode;
        vector<Node3D*> childNodes;

};

template <class NodeDim, class ObstacleDim>
class Space{
    public:
        Space();
        double L2(Node&, Node&);
        double L2(Node3D&, Node3D&);
        double L2(Obstacle&, Node&);
        double L2(Obstacle3D&, Node3D&);
        
        void directionComponent(Node&, Node&);
        void directionComponent(Node3D&, Node3D&);
        vector<ObstacleDim> obstacles;
        vector<NodeDim> nodes;
        void init();
        bool solve();
        bool checkCollision(NodeDim& node);
        NodeDim start;
        NodeDim goal;
        NodeDim& goalnode;
        NodeDim& addNode();
        NodeDim& getNearestNode(double& min_dist, NodeDim& currentNode, NodeDim& node);
        void addConnection(NodeDim& a, NodeDim& b);

        double delta;

};