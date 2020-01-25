#include "Node.hpp"
#include <cmath>
using namespace std;

bool IsLessNode::operator() (const VertexU_t& lhs, const VertexU_t& rhs)
{
	return (*m_graphPtr)[lhs]->GetWeight() > (*m_graphPtr)[rhs]->GetWeight();
}

IsLessNode::GraphU_t* IsLessNode::m_graphPtr = NULL;

Node::Node(int nDof, vector<double> angles): 
m_nDof(nDof),
m_cost(0),
m_nodeBackPtr(NULL)
{
	if (angles.size() != m_nDof)
	throw runtime_error("Incorrect number of angles length being set! \n");

	m_angles = angles;
	m_weight = numeric_limits<double>::infinity();
}

Node::~Node()
{

}

int Node::GetDof()
{
	return m_nDof;
}

void Node::SetDof(int nDof)
{
	m_nDof = nDof;
}

void Node::SetAngles(vector<double> angles)
{
	if (angles.size() != m_nDof)
		throw runtime_error("Incorrect number of angles length being set! \n");
	m_angles = angles;
}

vector<double> Node::GetAngles()
{
	return m_angles;
}

void Node::SetCost(double cost)
{
	m_cost = cost;
}

double Node::GetCost()
{
	return m_cost;
}

void Node::SetVisited()
{
	m_isVisited = true;
}

void Node::SetWeight(double weight)
{
	m_weight = weight;
}

double Node::GetWeight() const
{
	return m_weight;
}

void Node::UnsetVisited()
{
	m_isVisited = false;
}

bool Node::IsVisited()
{
	return m_isVisited;
}

void Node::SetNodeBackPtr(Node* node)
{
	//unique_lock<mutex> lock(m_mutex);
	m_nodeBackPtr = node;
}

Node* Node::GetNodeBackPtr()
{
	return m_nodeBackPtr;
}

void Node::PrintAnglesDeg()
{	
	for (auto it = m_angles.begin(); it != m_angles.end(); ++it)
	{
		cout << (*it)*(180/M_PI) << " , ";
	}
	cout << endl;
}

void Node::PrintNode()
{	
	for (auto it = m_angles.begin(); it != m_angles.end(); ++it)
	{
		cout << *it << " , ";
	}
	cout << "| Cost:  " << m_cost << endl;
	cout << "| Weight " << m_weight << endl;
}
