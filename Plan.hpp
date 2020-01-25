#ifndef PLAN_HPP
#define PLAN_HPP

#include <boost/graph/adjacency_list.hpp>
#include <chrono>
#include <ctime>
class Node;
// class Edge;

class Plan
{

public:
	Plan(int nDof);
	virtual ~Plan();
	void SetStartState(const std::vector<double>& startAngles);
	void SetGoalState(const std::vector<double>& goalAngles);
    virtual void BuildRoadmap();
	virtual bool RunPlanner() = 0;
    // virtual void RunTest(int numTests, double*** plan, int* planLength) = 0;
    virtual void GetPlan(std::vector<std::vector<double>>& path) = 0;
    void SaveStartGoalPairs(int numPairs);
protected:
    typedef boost::adjacency_list<boost::listS, boost::vecS, boost::bidirectionalS, Node*> Graph_t;
    typedef Graph_t::vertex_descriptor Vertex_t;
    typedef Graph_t::edge_descriptor Edge_t;

    double angleDifference(double angle1, double angle2);
    double wrapAngle(double angle);
    double wrapAngle02pi(double angle);

    double vectorNorm(std::vector<double>::iterator first, std::vector<double>::iterator last);
    double calculateVectorNorm();
    double getRandomNumberBetween(double min, double max);
    template<typename dataType> dataType* getCArrayFromVec(std::vector<dataType> vec);
    bool isValidConfiguration(std::vector<double> nodeAngles);
    Node* collisionFree(Node* node1, Node* node2, bool& isCollision);
    Node* collisionFree(Node* node1, Node* node2, int numSamples, bool& isCollision);
    Node* sampleNodeUniform();
    Node* sampleNodeGoalBias(double goalSampleProbability, Node* goal);
    Node* sampleNodeGoalBiasGaussian(double goalSampleProbability, double gaussSampleProbability, Node* goal);

    template<typename GraphType> Vertex_t addNodeToGraph(Node* node, GraphType& graph)
    {
        Vertex_t v = add_vertex(graph);
        graph[v] = node;
        return v;
    };

    Vertex_t getNearestNeighbor(Node* node, Graph_t& graph);
    std::vector<Vertex_t> getNeighborVertices(Node* node, double radius, Graph_t& graph);
    double calculateNodeDistance(Node* node1, Node* node2);
    Edge_t addEdge(Vertex_t vertex1, Vertex_t vertex2, Graph_t& graph);
    Edge_t addEdge(Vertex_t vertex1, Vertex_t vertex2, double cost, Graph_t& graph);

    void removeEdge(Vertex_t vertex1, Vertex_t vertex2, Graph_t& graph);
    double getEdgeCost(Node* node1, Node* node2);

    template<typename GraphType> Node* getNodePtrFromVertex(Vertex_t vertex, GraphType& graph)
    {
        return graph[vertex];
    };

    int getMapIndex(int x, int y);

    void readStartsGoalsFromFile(std::string startsFile, std::string goalsFile, int numPairs);

    void printGraph(Graph_t& graph);
    void printGraphToFile(Graph_t& graph);
    void printStats(std::string filename, int numTests);

    void clearGraph(Graph_t& graph);

    double* m_map;
    int m_xSize;
    int m_ySize;
    int m_nDof;
    double m_jointLowerLimit;
    double m_jointUpperLimit;

    std::vector<double> m_startState;
    std::vector<double> m_goalState;
    Node* m_startNode;
    Node* m_goalNode;
    Vertex_t m_startVertex;
    Vertex_t m_goalVertex;
    std::vector<std::vector<double>> m_startStateVec;
    std::vector<std::vector<double>> m_goalStateVec;

    Graph_t m_graph;

    std::vector<double> m_planningTime;
    std::vector<int> m_numSamples;
    std::vector<double> m_pathCost;
    std::vector<int> m_startGoalPairIndex;
    std::string m_statsFileName;


private:
    std::random_device m_randomDevice;



};


#endif