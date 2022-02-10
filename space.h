#include <vector>

using std::vector;

struct Obstacle{
    public:
        double x;
        double y;
        double r;
};

struct Node{
    public:
        double x;
        double y;
        Node* parentNode;
        vector<Node*> childNodes;

};

class Space{
    public:
        vector<Obstacle> obstacles;
        vector<Node> nodes;
        void init();
        bool solve();
        bool checkCollision(Node& node);
        Node start;
        Node goal;
        Node& addNode();
        Node& getNearestNode(double& min_dist, Node& currentNode, Node& node);
        void addConnection(Node& a, Node& b);

        double delta;

};