#ifndef __NFF_HARP_ADAPTER__
#define __NFF_HARP_ADAPTER__

#include "nff_heuristic_adapter.hxx"
#include "nff_rp_heuristic.hxx"
#include "nff_best_supporter.hxx"
#include "nff_helpful.hxx"

namespace NFF
{

/* HARP - helpful actions from relaxed plan */
class HARPHeuristicAdapter : public HeuristicAdapter 
{
protected:
    	BestSupporter *bs;
	HelpfulActionExtractor *haextractor;
	RelaxedPlanHeuristic *heuristic;
	
	bool 		use_min_cost;
	static float 	min_cost;
	unsigned	m_num_obs_accounted;
	RelaxedPlan	m_relaxed_plan;
public:

     	HARPHeuristicAdapter(BestSupporter *bs, HelpfulActionExtractor *hae);
	virtual ~HARPHeuristicAdapter(); 
	virtual bool suggestsHelpful() { return true; }
	virtual float eval( State &s );
	virtual float eval( State &s, std::vector<unsigned> &helpful );
	virtual float eval( State &s, std::vector<unsigned> &helpful, std::vector<unsigned>& partial_plan );
	unsigned num_obs_accounted() { return m_num_obs_accounted; }
	RelaxedPlan& relaxed_plan() { return m_relaxed_plan; }
};

}

#endif // nff_harp_adapter.hxx
