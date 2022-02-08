#include <vector>

using std::vector;

struct Obstacle{
    public:
        int x;
        int y;
        int r;
};

class Node{
    public:
        int x;
        int y;
        Node* parentNode;
        Node* childNode;

};

class Space{
    public:
        vector<Obstacle> obstacles;
        vector<Node> Nodes;
        void init();
        bool solve();
        bool checkCollision(Node node);
        Node start;
        Node goal;
        Node samplePoint();
        void addNode();
        void addConnection(Node a, Node b);

        double delta;

};