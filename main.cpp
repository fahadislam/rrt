#include "Rrt.hpp"
#include <cmath>

int main(int argc, char* argv[])
{
	int numofDOFs = 4;
	Plan* planner = new Rrt(numofDOFs);

	std::vector<double> armstart_anglesV_rad = {M_PI, M_PI, M_PI, M_PI};
	std::vector<double> armgoal_anglesV_rad = {M_PI, M_PI/2, M_PI/2, M_PI};
	planner->SetStartState(armstart_anglesV_rad);
	planner->SetGoalState(armgoal_anglesV_rad);
	planner->RunPlanner();

	std::vector<std::vector<double>> path;	
	planner->GetPlan(path);
}