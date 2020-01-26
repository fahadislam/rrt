#ifndef RRT_HPP
#define RRT_HPP
#include "Plan.hpp"

#include "kdtree.h"

class Rrt : public Plan
{

public:
	Rrt(int nDof);
	virtual ~Rrt();
	void SetMaxIterations(int N);
	virtual bool RunPlanner();
    // virtual void RunTest(int numTests, double*** plan, int* planLength);
    virtual void GetPlan(std::vector<std::vector<double>>& path);

protected:
    Node* extend(Node* sampledNode, Node* nearestNeighborNode);
    std::vector<std::vector<double>> findPathFromVertexAToB(Vertex_t vertexA, Vertex_t vertexB, Graph_t& graph);
    std::vector<Vertex_t> getChildren(Vertex_t vertex, Graph_t& graph);
	int m_maxIterations;
	double m_goalBiasProbability;
	double m_eps;
	double m_terminationDistance;
private:	
    std::string m_statsFileName;


};






#endif