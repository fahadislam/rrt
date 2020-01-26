#include "Plan.hpp"
#include "Node.hpp"
#include <math.h>
#include <random>
#include <fstream>
#include <iostream>
#include <limits>
#include <fstream>
#include <stdlib.h>     /* srand, rand */

using namespace std;
using namespace boost;

// int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
// 		   int x_size, int y_size);

Plan::Plan(int nDof):
m_nDof(nDof)
{
	m_jointLowerLimit = -M_PI;
	m_jointUpperLimit = M_PI;
	srand (100);

}

Plan::~Plan()
{
	clearGraph(m_graph);
}

void Plan::SetStartState(const std::vector<double>& startAngles)
{
	for (int i = 0; i < m_nDof; i++)
		m_startState.push_back(startAngles[i]);

	if (!isValidConfiguration(m_startState))
		throw runtime_error("Start state is not valid!");

	m_startNode = new Node(m_nDof, m_startState);
}

void Plan::SetGoalState(const std::vector<double>& goalAngles)
{
	for (int i = 0; i < m_nDof; i++)
		m_goalState.push_back(goalAngles[i]);

	if (!isValidConfiguration(m_goalState))
		throw runtime_error("Goal state is not valid!");

	m_goalNode = new Node(m_nDof, m_goalState);
}

void Plan::BuildRoadmap()
{

}

void Plan::SaveStartGoalPairs(int numPairs)
{
	ofstream foutS, foutG;
	foutS.open("starts.txt");
	foutG.open("goals.txt");

	for (int i = 0; i < numPairs; ++i)
	{
		Node* nodeStart = sampleNodeUniform();
		Node* nodeGoal = sampleNodeUniform();

		vector<double> anglesStart = nodeStart->GetAngles();
		vector<double> anglesGoal = nodeGoal->GetAngles();

		for (int j = 0; j < m_nDof; ++j)
		{
			foutS << anglesStart[j] << ' ';
			foutG << anglesGoal[j] << ' ';
		}
		foutS << endl;
		foutG << endl;		
	}
	foutS.close();
	foutG.close();
}


/**
 * @brief      angle1-angle2
 *
 * @param[in]  angle1  The angle 1
 * @param[in]  angle2  The angle 2
 *
 * @return     { description_of_the_return_value }
 */
double Plan::angleDifference(double angle1, double angle2)
{
    // double diff = fmod(( angle2 - angle1 + 180 ),360) - 180;
    // return diff < -180 ? diff + 360 : diff;
    return wrapAngle(angle1-angle2);
    // return angle1-angle2;

}

/**
 * @brief      Normalize an angle into the range [-pi, pi].
 *
 * @param[in]  angle  The angle
 *
 * @return     The angle is radians.
 */
double Plan::wrapAngle(double angle)
{
    // normalize to [-2*pi, 2*pi] range
    if (std::fabs(angle) > 2.0 * M_PI) {
        angle = std::fmod(angle, 2.0 * M_PI);
    }

    if (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    if (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }

    return angle;
}

double Plan::wrapAngle02pi(double angle)
{
	return wrapAngle(angle)+M_PI;
}

/**
 * @brief      { function_description }
 *
 * @param[in]  first  The first
 * @param[in]  last   The last
 *
 * @return     { description_of_the_return_value }
 */
double Plan::vectorNorm(vector<double>::iterator first, vector<double>::iterator last) 
{
  return sqrt(inner_product(first, last, first, 0.0));
}

/**
 * @brief      Gets the random number between.
 *
 * @param[in]  min   The minimum
 * @param[in]  max   The maximum
 *
 * @return     The random number between.
 */
double Plan::getRandomNumberBetween(double min, double max)
{
	// Implement best random number generation
	// ((double) rand() / (RAND_MAX))
	mt19937 gen(m_randomDevice());
	uniform_real_distribution<double> distr(min, max);
	return distr(gen);
}

template<typename dataType> dataType* Plan::getCArrayFromVec(vector<dataType> vec)
{
	dataType* cArray = new dataType(vec.size());
	for (int i = 0; i < vec.size(); ++i)
	{
		cArray[i] = vec[i];
	}
	return cArray;
}

bool Plan::isValidConfiguration(vector<double> nodeAngles)
{
	double* angles = new double[m_nDof];

	for (int i = 0; i < m_nDof; ++i)
		angles[i] = nodeAngles[i]; 
	
	bool dec = true;
	// bool dec = IsValidArmConfiguration(angles, m_nDof, m_map, m_xSize, m_ySize);
	delete[] angles;
	return dec;
}

Node* Plan::collisionFree(Node* node1, Node* node2, bool& isCollision)
{
	vector<double> node1Angles = node1->GetAngles();
	vector<double> node2Angles = node2->GetAngles();

	double distance = 0;
	for (int i = 0; i < m_nDof; i++)
	{
        if(distance < fabs(node2Angles[i] - node1Angles[i]))
            distance = fabs(node2Angles[i] - node1Angles[i]);
    }

    int numSamples = (int)(distance/(M_PI/180));

    return collisionFree(node1, node2, numSamples, isCollision);
}


/**
 * @brief      Checks if the path from node1 to node2 is collision free
 * 				by repeatedly calling isValidConfiguration on the linear
 * 				interpolation between the two nodes.
 *
 * @param      node1  The node 1
 * @param      node2  The node 2
 *
 * @return     { description_of_the_return_value }
 */
Node* Plan::collisionFree(Node* node1, Node* node2, int numSamples, bool& isCollision)
{

	vector<double> node1Angles = node1->GetAngles();
	vector<double> node2Angles = node2->GetAngles();
	vector<double> currAngles(m_nDof);
	vector<double> lastValidConfig = node1Angles; 
	Node* nodeExtended;

	if (!isValidConfiguration(node1->GetAngles()))
		throw runtime_error("node1 not valid in collisionFree");

	for (int i = 0; i < numSamples; ++i)
	{
		for (int j = 0; j < m_nDof; ++j)
		{
			currAngles[j] = node1Angles[j] + ((double)(i)/(numSamples-1))*angleDifference(node2Angles[j],node1Angles[j]);/*/angleDiffNorm);*/
		}

		if(!isValidConfiguration(currAngles))
		{
			isCollision = true;
	
			if (!isValidConfiguration(lastValidConfig))
				throw runtime_error("lastValidConfig not valid in collisionFree");


			nodeExtended = new Node(m_nDof, lastValidConfig);
			return nodeExtended;
		}

		lastValidConfig = currAngles;
	}


	if (!isValidConfiguration(node2->GetAngles()))
	{
		throw runtime_error("node2 not valid in collisionFree");
	}

	isCollision = false;
	return node2;
}

Node* Plan::sampleNodeUniform()
{
	Node* sampledNode;
	vector<double> sampledAngles;
	
	bool validConfig = false;

	while (!validConfig)
	{		
		sampledAngles.clear();
		for (int i = 0; i < m_nDof; ++i)
			sampledAngles.push_back(getRandomNumberBetween(m_jointLowerLimit, m_jointUpperLimit));
		
		// Check if the sampled state is in free space
		if (isValidConfiguration(sampledAngles))
		   validConfig = true;		
		
	}

	sampledNode = new Node(m_nDof, sampledAngles);

	return sampledNode;
}

Node* Plan::sampleNodeGoalBias(double goalSampleProbability, Node* goal)
{
	double r = getRandomNumberBetween(0,1);

	Node* sampledNode;
	if (r < goalSampleProbability)
	{
		sampledNode = new Node(m_nDof, goal->GetAngles());
	}
	else
	{			
		sampledNode = sampleNodeUniform();
	}

	return sampledNode;
}

Node* Plan::sampleNodeGoalBiasGaussian(double goalSampleProbability, double gaussSampleProbability, Node* goal)
{
	double r = getRandomNumberBetween(0,1);
	mt19937 gen(m_randomDevice());
	normal_distribution<double> normal_dist(0, 0.5);

	Node* sampledNode;
	if (r < goalSampleProbability)
	{
		sampledNode = new Node(m_nDof, goal->GetAngles());
	}
	else if ((r >= goalSampleProbability) && (r < gaussSampleProbability))
	{			
		vector<double> angles = goal->GetAngles();
		for (vector<double>::iterator it = angles.begin(); it != angles.end(); ++it)
		{
			*it = wrapAngle02pi(*it + normal_dist(gen)); 
		}
		sampledNode = new Node(m_nDof, angles);
	}
	else
	{
		sampledNode = sampleNodeUniform();

	}

	return sampledNode;
}

Vertex_t Plan::getNearestNeighbor(Node* node, Graph_t& graph, kdtree* kd)
{
	printf("=============Finding NN================\n");
	// kdres* q_near_kd = kd_nearest(kd, &node->GetAngles()[0]);
	kdnode* q_near_kd = kd_nearest_node(kd, &node->GetAngles()[0]);
	// Vertex_t* q_near_n = (Vertex_t*) kd_res_item_data(q_near_kd);
	// Vertex_t* q_near_n = (Vertex_t*) q_near_kd->data;
	Vertex_t q_near_n =  q_near_kd->graph_vertex;
	// kd_res_free(q_near_kd);

	//=========================================================

	Graph_t::vertex_iterator itVBegin, itVEnd; 
	tie(itVBegin, itVEnd) = vertices(graph);
	
	double minDist = numeric_limits<double>::infinity();
	Vertex_t nearestNeigbor;

	// if (num_vertices(graph) == 0)
	// 	cout << "Graph Empty! \n";


	for (Graph_t::vertex_iterator it = itVBegin; it != itVEnd; ++it)
	{	
		double dist = calculateNodeDistance(getNodePtrFromVertex(*it, graph), node);
		if (dist < minDist)
		{
			minDist = dist;
			nearestNeigbor = *it;
		}
	}

	if (nearestNeigbor != q_near_n) {

		printGraph(m_graph);
		printf("***Mismatch***\n");
		printf("node:\n");
		node->PrintNode();
		printf("brute force nn:\n");
		getNodePtrFromVertex(nearestNeigbor, graph)->PrintNode();
		printf("kdtree nn:\n");
		getNodePtrFromVertex(q_near_n, graph)->PrintNode();
		printf("kdnode pos : %f", q_near_kd->pos[0]);
		getchar();
	}
	else {
		printGraph(m_graph);
		printf("***Right***\n");
		printf("node:\n");
		node->PrintNode();
		printf("brute force nn:\n");
		getNodePtrFromVertex(nearestNeigbor, graph)->PrintNode();
		printf("kdtree nn:\n");
		getNodePtrFromVertex(q_near_n, graph)->PrintNode();
	}

	return q_near_n;
	// return nearestNeigbor;
}

std::vector<Vertex_t> Plan::getNeighborVertices(Node* node, double radius, Graph_t& graph)
{
	Graph_t::vertex_iterator itVBegin, itVEnd; 
	tie(itVBegin, itVEnd) = vertices(graph);
	
	vector<Vertex_t> nearestNeigborVector;
	
	if (num_vertices(graph) == 0)
		cout << "Graph Empty! \n";

	for (Graph_t::vertex_iterator it = itVBegin; it != itVEnd; ++it)
	{
		double dist = calculateNodeDistance(getNodePtrFromVertex(*it, graph), node);
		if (dist < radius)
		{
			nearestNeigborVector.push_back(*it);
		}
	}
	return nearestNeigborVector;	
}

double Plan::calculateNodeDistance(Node* node1, Node* node2)
{
	vector<double> node1Angles = node1->GetAngles();
	vector<double> node2Angles = node2->GetAngles();

	double dist=0;
	for (int i = 0; i < m_nDof; ++i)
	{
		dist += pow(angleDifference(node1Angles[i],node2Angles[i]),2);
	}
	// dist = sqrt(dist);
	return dist;
}

/**
 * @brief      Adds an edge vertexSource---->vertexDest
 *
 * @param[in]  vertexSource  The vertex source
 * @param[in]  vertexDest    The vertex destination
 *
 * @return     { description_of_the_return_value }
 */
Edge_t Plan::addEdge(Vertex_t vertexSource, Vertex_t vertexDest, Graph_t& graph)
{
	Node* nodeSource = getNodePtrFromVertex(vertexSource, graph);
	Node* nodeDest = getNodePtrFromVertex(vertexDest, graph);
	nodeDest->SetCost(nodeSource->GetCost() + getEdgeCost(nodeSource, nodeDest));

	pair<Edge_t, bool> edgePair = add_edge(vertexSource, vertexDest, graph);
	return edgePair.first;
}

Edge_t Plan::addEdge(Vertex_t vertexSource, Vertex_t vertexDest, double cost, Graph_t& graph)
{
	getNodePtrFromVertex(vertexDest, graph)->SetCost(cost);
	pair<Edge_t, bool> edgePair = add_edge(vertexSource, vertexDest, graph);
	return edgePair.first;
}

void Plan::removeEdge(Vertex_t vertex1, Vertex_t vertex2, Graph_t& graph)
{
	remove_edge(vertex1, vertex2, graph);
}

double Plan::getEdgeCost(Node* node1, Node* node2)
{
	double cost = calculateNodeDistance(node1, node2);
	return cost;
}

int Plan::getMapIndex(int x, int y)
{
	return y*m_xSize+x;
}

void Plan::readStartsGoalsFromFile(std::string startsFile, std::string goalsFile, int numPairs)
{
	m_startStateVec.clear();
	m_goalStateVec.clear();
	ifstream finG, finS;
	finS.open(startsFile.c_str());
	finG.open(goalsFile.c_str());
	double valG, valS;

	for (int i = 0; i < numPairs; ++i)
	{
		vector<double> start, goal; 
		for (int j = 0; j < m_nDof; ++j)
		{
			finS >> valS;
			finG >> valG;
			start.push_back(valS);
			goal.push_back(valG);
		}
		m_startStateVec.push_back(start);
		m_goalStateVec.push_back(goal);
	}
}

void Plan::printGraph(Graph_t& graph)
{
	Graph_t::vertex_iterator itVBegin, itVEnd; 
	tie(itVBegin, itVEnd) = vertices(graph);
	
	
	if (num_vertices(graph) == 0)
		cout << "Graph Empty! \n";

	cout << "*******************GRAPH***************" << endl;
	int i = 1;
	for (Graph_t::vertex_iterator it = itVBegin; it != itVEnd; ++it)
	{
		cout << "Vertex " << i << ':'<< endl;
		getNodePtrFromVertex(*it, graph)->PrintNode(); 
		i++;
	}
	cout << "**************************************" << endl;

}

void Plan::printGraphToFile(Graph_t& graph)
{
	Graph_t::vertex_iterator itVBegin, itVEnd; 
	tie(itVBegin, itVEnd) = vertices(graph);
	
	
	if (num_vertices(graph) == 0)
		cout << "Graph Empty! \n";

	ofstream fout;
	fout.open("graph.txt");
	
	for (Graph_t::vertex_iterator it = itVBegin; it != itVEnd; ++it)
	{


		double x= ((double)m_xSize)/2.0;
		double y = 0;
		vector<double> angles = getNodePtrFromVertex(*it, graph)->GetAngles();
		for (int i = 0; i < m_nDof; ++i)
		{
			x = x + 10*cos(angles[i]);
			y = y + 10*sin(angles[i]);
		}
		fout << x << ' ' << y <<endl;
	}

	fout.close();
}

void Plan::printStats(string filename, int numTests)
{
	ofstream fout;
	fout.open(filename.c_str());
	cout << "Printing stats to file: " << filename << endl;
	for (int i = 0; i < m_planningTime.size(); ++i)
	{
		fout << m_startGoalPairIndex[i] << ' ' << m_planningTime[i] << ' ' << m_numSamples[i] << ' ' << m_pathCost[i] << endl;
	}
	double meanPlanningTime = accumulate(m_planningTime.begin(), m_planningTime.end(), 0.0)/m_planningTime.size(); 
	double meanNumSamples = accumulate(m_numSamples.begin(), m_numSamples.end(), 0.0)/m_numSamples.size(); 
	double meanPathCost = accumulate(m_pathCost.begin(), m_pathCost.end(), 0.0)/m_pathCost.size(); 

	fout << meanPlanningTime << ' ' << meanNumSamples << ' ' << meanPathCost << endl;
}

void Plan::clearGraph(Graph_t& graph)
{

}