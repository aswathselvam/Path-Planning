#include <vector>

using std::vector;

struct Obstacle{
    public:
        int x;
        int y;
        int r;
};

struct Node{
    public:
        int x;
        int y;
        Node* parentNode;
        Node* childNode;

};

class Space{
    public:
        vector<Obstacle> obstacles;
        vector<Node> nodes;
        void init();
        bool solve();
        bool checkCollision(Node node);
        Node start;
        Node goal;
        Node addNode();
        Node getNearestNode(Node node);
        void addConnection(Node a, Node b);

        double delta;

};