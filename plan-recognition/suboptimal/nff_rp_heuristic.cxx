#include "nff_rp_heuristic.hxx"

namespace NFF
{

RelaxedPlanHeuristic::RelaxedPlanHeuristic(BestSupporter &b) 
	: bs(b) 
{ 
}

RelaxedPlanHeuristic::~RelaxedPlanHeuristic()
{
}

float RelaxedPlanHeuristic::eval( State &s, RelaxedPlan &rp ) 
{
    	bs.computeSupports(s);
	bs.extractPlan(s, rp);
	return rp.cost();
}

float RelaxedPlanHeuristic::eval( State &s, RelaxedPlan &rp, std::vector<unsigned>& partial_plan ) 
{
    	bs.computeSupports(s, partial_plan);
	bs.extractPlan(s, rp);
	return rp.cost();
}


}
