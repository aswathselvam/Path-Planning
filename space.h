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
        Node(double, double, double);
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
        // Space();
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
        NodeDim& addNode();
        NodeDim& getNearestNode(double& min_dist, NodeDim& currentNode, NodeDim& node);
        void addConnection(NodeDim& a, NodeDim& b);

        double delta;

};