#ifndef __NFF_RP_HEURISTIC__
#define __NFF_RP_HEURISTIC__

#include "nff_heuristic.hxx"
#include "nff_best_supporter.hxx"
#include "nff_relaxed_plan.hxx"

namespace NFF
{

class RelaxedPlanHeuristic : public Heuristic 
{

protected:
    	BestSupporter &bs;

public:
      	RelaxedPlanHeuristic(BestSupporter &b);

    	virtual ~RelaxedPlanHeuristic();

	virtual float eval( State &s ) 
	{
		RelaxedPlan dummy;
		return eval(s, dummy) ;
	}

	virtual float eval( State &s, RelaxedPlan &rp );

	virtual float eval( State &s, RelaxedPlan &rp, std::vector<unsigned>& partial_plan );

};

}

#endif // nff_rp_heuristic.hxx
