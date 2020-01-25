#ifndef NODE_HPP
#define NODE_HPP

#include <boost/graph/adjacency_list.hpp>
#include <vector>
#include <iostream>

class Node;

class IsLessNode
{
public:
    typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, Node*> GraphU_t;
    typedef GraphU_t::vertex_descriptor VertexU_t;
	bool operator() (const VertexU_t& lhs, const VertexU_t& rhs);

	static GraphU_t* m_graphPtr;
};

class Node
{

public:
	Node(int nDof, std::vector<double> angles);
	~Node();
    typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, Node*> GraphU_t;

	int GetDof();
	void SetDof(int nDof);
	void SetAngles(std::vector<double> angles);
	std::vector<double> GetAngles();
	void SetCost(double cost);
	double GetCost();
	void SetWeight(double weight);
	double GetWeight() const;
	void SetVisited();
	void UnsetVisited();
	bool IsVisited();
	void SetNodeBackPtr(Node* node);
	Node* GetNodeBackPtr();

	void PrintAnglesDeg();
	void PrintNode();

protected:

private:
	std::vector<double> m_angles;
	int m_nDof;
	double m_cost;
	double m_weight;
	bool m_isVisited;
	Node* m_nodeBackPtr;
}; 

#endif