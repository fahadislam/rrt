#include "Rrt.hpp"
#include "Node.hpp"
#include <math.h>
#include <vector>

using namespace std;

Rrt::Rrt(int nDof):Plan(nDof), 
m_maxIterations(10000),
m_statsFileName("Rrt_Stats.txt")
{
	m_goalBiasProbability = 0.05;
	m_eps = 1.0;
	// m_eps = 0.5;
	m_terminationDistance = 0.5;
}

Rrt::~Rrt()
{

}

void Rrt::SetMaxIterations(int N)
{
	m_maxIterations = N;
}

bool Rrt::RunPlanner()
{
	cout << "**************** Running RRT ************* \n";
	auto start = std::chrono::system_clock::now();
	m_startVertex = addNodeToGraph(m_startNode, m_graph);
	for (int i = 0; i < m_maxIterations; ++i)
	{
		if (i%1000 == 0)
			cout << "Iteration number: " << i << " | Graph size: " << num_vertices(m_graph) << endl;
		Node* sampledNode = sampleNodeGoalBias(m_goalBiasProbability, m_goalNode);
		// Node* sampledNode = sampleNodeGoalBiasGaussian(m_goalBiasProbability, 0.25, m_goalNode);

		Vertex_t nearestNeighborVertex = getNearestNeighbor(sampledNode, m_graph);

		Node* newNode = extend(sampledNode, getNodePtrFromVertex(nearestNeighborVertex, m_graph));
		
		bool isCollision = false;
		newNode = collisionFree(getNodePtrFromVertex(nearestNeighborVertex, m_graph), newNode, isCollision);
		
		

		Vertex_t newNodeVertex = addNodeToGraph(newNode, m_graph);
		addEdge(nearestNeighborVertex, newNodeVertex, m_graph);
		double distToGoal = calculateNodeDistance(newNode,m_goalNode);
		if (distToGoal < m_terminationDistance)
		{
			cout << "Goal reached with distance: "<< distToGoal << "Iterations : "<< i <<'\n';
			auto end = std::chrono::system_clock::now();
			std::chrono::duration<double> elapsed_seconds = end - start;
			m_planningTime.push_back(elapsed_seconds.count());
			m_numSamples.push_back(i+1);
			return true;
		}
	}
	return false;
}

// void Rrt::RunTest(int numTests, double*** plan, int* planLength)
// {
// 	readStartsGoalsFromFile("starts.txt", "goals.txt", 20);
// 	int succRuns = 1;
// 	int runCount = 1;
// 	while (succRuns <= numTests)
// 	{
// 		cout << "Test count: " << succRuns << endl;

// 		m_startNode = new Node(m_nDof, m_startStateVec[succRuns-1]);

// 		m_goalNode = new Node(m_nDof, m_goalStateVec[succRuns-1]);
// 		cout << "Start States: ";
// 		m_startNode->PrintNode();
// 		cout << "Goal States: ";
// 		m_goalNode->PrintNode();	

// 		bool bSucc = RunPlanner();
// 		runCount++;

// 		if (bSucc)
// 		{
// 			runCount = 1;
// 			m_startGoalPairIndex.push_back(succRuns);
// 			GetPlan(plan, planLength);
// 			succRuns++;
		
// 		}
	
// 		if (runCount > 1)
// 		{
// 				runCount = 1;
// 				succRuns++;
// 		}		
	
// 		cout << "runCount: " <<runCount << endl;;
		
// 		m_graph.clear();
// 	}
// 	cout << "Number of successful runs: " << m_startGoalPairIndex.size() << endl;
// 	cout << "planningTimeVec size: " << m_planningTime.size() << endl;
// 	cout << "numSamples size: " << m_numSamples.size() << endl;
// 	cout << "pathCost size: " << m_pathCost.size() << endl;
// 	printStats(m_statsFileName, numTests);
// }


/**
 * @brief      Gets the plan.
 *
 * @param      plan        The plan
 * @param      planLength  The plan length
 */
void Rrt::GetPlan(std::vector<std::vector<double>>& path)
{
	Vertex_t nearestVertexToGoal = getNearestNeighbor(m_goalNode, m_graph);
	
	cout << "Start Node: ";
	m_startNode->PrintNode();
	
	cout << "Goal Node: ";
	m_goalNode->PrintNode();
	
	cout << "Final vertex in path: "; 
	(getNodePtrFromVertex(nearestVertexToGoal, m_graph))->PrintNode();

	double pathCost = getNodePtrFromVertex(nearestVertexToGoal, m_graph)->GetCost();

	cout << "Path cost: " << pathCost << endl;
	m_pathCost.push_back(pathCost);

	path = findPathFromVertexAToB(m_startVertex, nearestVertexToGoal, m_graph);

	// *planLength = path.size();
	// *plan = new double*[*planLength];
	// for (int i = 0; i < *planLength; ++i)
	// {
	// 	(*plan)[i] = new double[m_nDof];
	// 	for (int j = 0; j < m_nDof; ++j)
	// 	{
	// 		(*plan)[i][j] = path[i][j];
	// 	}
	// }
}

Node* Rrt::extend(Node* sampledNode, Node* nearestNeighborNode)
{
	vector<double> newNodeAngles(m_nDof);
	vector<double> sampledNodeAngles = sampledNode->GetAngles();
	vector<double> nearestNeighborNodeAngles = nearestNeighborNode->GetAngles();

	// if sampledNode is closer than m_eps, return that 
	if (calculateNodeDistance(sampledNode, nearestNeighborNode) < m_eps)
		return sampledNode;

	double angleDiffNorm = 0;
	for (int i = 0; i < m_nDof; ++i)
	{
		double diff = angleDifference(sampledNodeAngles[i],nearestNeighborNodeAngles[i]);
		angleDiffNorm += pow(diff,2);
	}
	angleDiffNorm = sqrt(angleDiffNorm);

	for (int i = 0; i < m_nDof; ++i)
	{
		newNodeAngles[i] = nearestNeighborNodeAngles[i] + m_eps*(angleDifference(sampledNodeAngles[i],nearestNeighborNodeAngles[i])/angleDiffNorm); 
	}
	
	Node* newNode = new Node(m_nDof, newNodeAngles);

	delete sampledNode;
	return newNode;	
}


vector<vector<double>> Rrt::findPathFromVertexAToB(Vertex_t vertexA, Vertex_t vertexB, Graph_t& graph)
{
	vector<vector<double>> path;

	Vertex_t currVertex = vertexB;
	path.insert(path.begin(), graph[currVertex]->GetAngles());
	
	while (currVertex != vertexA)
	{
		Graph_t::in_edge_iterator eItBegin, eItEnd;
		tie(eItBegin, eItEnd) = in_edges(currVertex, graph);

		if (in_degree(currVertex, graph) != 1)
		{
			cout << in_degree(currVertex, graph) << endl;
			throw runtime_error("More than one parent for a vertex in RRT, Something is wrong! \n");
		}

		Vertex_t parent = source(*eItBegin, graph);
		path.insert(path.begin(), graph[parent]->GetAngles());
		currVertex = parent;		
	}

	return path;
}

vector<Plan::Vertex_t> Rrt::getChildren(Vertex_t vertex, Graph_t& graph)
{
	Graph_t::out_edge_iterator eItBegin, eItEnd;
	tie(eItBegin, eItEnd) = out_edges(vertex, graph);
	vector<Vertex_t> children;

	for (Graph_t::out_edge_iterator it = eItBegin; it != eItEnd; ++it)
	{
		children.push_back(target(*it, graph));
	}
	return children;
}







