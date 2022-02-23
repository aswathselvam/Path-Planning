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
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <vector>

using std::vector;

class Obstacle {
 public:
  /**
   * @brief Default contstructor for Obstacle class.
   */
  Obstacle();

  /**
   * @brief Constructor for assigning ID of obstacle.
   * @param int ID of obstacle, or i-th obstacle.
   */
  Obstacle(int);

  double x;          ///< X coordinate of obstacle.
  double y;          ///< Y coordinate of obstacle.
  double r;          ///< Radius of obstacle.
  bool intersected;  ///< Flag for setting if obstacle is intersected with any
                     ///< of the sampled nodes.
};

class Obstacle3D {
 public:
  /**
   * @brief Constructor for assigning ID of obstacle.
   * @param int ID of obstacle, or i-th obstacle.
   */
  Obstacle3D(int);
  double x;          ///< X coordinate of obstacle.
  double y;          ///< Y coordinate of obstacle.
  double z;          ///< Z coordinate of obstacle.
  double r;          ///< Radius of obstacle.
  bool intersected;  ///< Flag for setting if obstacle is intersected with any
                     ///< of the sampled nodes.
};

class Node {
 public:
  /**
   * @brief Set coordinates of the Node.
   * @param x X coordinate of Node.
   * @param y Y coordinate of Node.
   * @param dummy Dummy variable.
   */
  void setup(double x, double y, double dummy);
  double x;                  ///< X coordinate of Node.
  double y;                  ///< Y coordinate of Node.
  Node* parentNode;          ///< Pointer to Parent Node.
  vector<Node*> childNodes;  ///< Vector of Pointers to Child Nodes.
};

class Node3D {
 public:
  /**
   * @brief Set coordinates of the Node.
   * @param x X coordinate of Node.
   * @param y Y coordinate of Node.
   * @param z Z coordinate of Node.
   */
  void setup(double x, double y, double z);
  double x;                    ///< X coordinate of Node.
  double y;                    ///< Y coordinate of Node.
  double z;                    ///< Z coordinate of Node.
  Node3D* parentNode;          ///< Pointer to Parent Node.
  vector<Node3D*> childNodes;  ///< Pointer to Child Nodes.
};

template <class NodeDim, class ObstacleDim>
class Space {
 public:
  /**
   * @brief Default contstructor for Space class.
   */
  Space();

  /**
   * @brief Calculate Euclidean distance between Obstacle and Node.
   * @param Node Reference to 2D Node object.
   * @param Node Reference to 2D Node object.
   * @return Distance between the 2D Node and a 2D Node.
   */
  double L2(Node&, Node&);

  /**
   * @brief Calculate Euclidean distance between Obstacle and Node.
   * @param Node3D Reference to 3D Node object.
   * @param Node3D Reference to 3D Node object.
   * @return Distance between the 3D Node and a 3D Node.
   */
  double L2(Node3D&, Node3D&);

  /**
   * @brief Calculate Euclidean distance between Obstacle and Node.
   * @param Obstacle Reference to 2D Obstacle object.
   * @param Node Reference to 2D Node object.
   * @return Distance between the 2D Obstacle and a 2D Node.
   */
  __host__ __device__ double L2(Obstacle&, Node&);

  /**
   * @brief Calculate Euclidean distance between Obstacle and Node.
   * @param Obstacle3D Reference to 3D Obstacle object.
   * @param Node3D Reference to 3D Node object.
   * @return Distance between the 3D Obstacle and a 3D Node.
   */
  double L2(Obstacle3D&, Node3D&);

  /**
   * @brief Assigns values to a node.
   * @param Node 
   * @param Node 
   */
  void directionComponent(Node&, Node&);

  /**
   * @brief Calculate Euclidean distance between Obstacle and Node.
   * @param Node3D Reference to 3D Obstacle object.
   * @param Node3D Reference to 3D Node object.
   * @return Distance between the 3D Obstacle and a 3D Node.
   */
  void directionComponent(Node3D&, Node3D&);


  vector<ObstacleDim> obstacles;
  ObstacleDim* h_obstacles;
  ObstacleDim* d_obstacles;
  vector<NodeDim> nodes;
  void init();
  bool solve();
  bool checkCollision(NodeDim& node);
  NodeDim start;
  NodeDim goal;
  NodeDim& goalnode;
  NodeDim& addNode();
  NodeDim& getNearestNode(double& min_dist, NodeDim& currentNode,
                          NodeDim& node);
  void addConnection(NodeDim& a, NodeDim& b);

  int width;
  int height;
  double delta;
  int N_OF_OBSTACLES;
};

template <class NodeDim, class ObstacleDim>
__global__ void cudaCheckCollision(Space<NodeDim, ObstacleDim>* spacep,
                                   Obstacle* d_obstacles, Node* node);
