#include <vector>

using std::vector;

struct Obstacle{
    public:
        double x;
        double y;
        double r;
};

struct Obstacle3D{
    public:
        double x;
        double y;
        double z;
        double r;
};

struct Node{
    public:
        double x;
        double y;
        Node* parentNode;
        vector<Node*> childNodes;

};

struct Node3D{
    public:
        double x;
        double y;
        double z;
        Node* parentNode;
        vector<Node*> childNodes;

};

template <class NodeDim>
class Space{
    public:
        // Space();
        double L2(Node&, Node&);
        double L2(Node3D&, Node3D&);
        double L2(Obstacle&, Node&);
        double L2(Obstacle3D&, Node3D&);

        vector<Obstacle> obstacles;
        vector<NodeDim> nodes;
        void init();
        bool solve();
        bool checkCollision(Node& node);
        bool checkCollision(Node3D& node);
        NodeDim start;
        NodeDim goal;
        NodeDim& addNode();
        NodeDim& getNearestNode(double& min_dist, NodeDim& currentNode, NodeDim& node);
        void addConnection(NodeDim& a, NodeDim& b);

        double delta;

};