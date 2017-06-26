#include "nff_harp_adapter.hxx"
#include <limits>

namespace NFF
{

float HARPHeuristicAdapter::min_cost = 0.01;

HARPHeuristicAdapter::HARPHeuristicAdapter(BestSupporter *bs, HelpfulActionExtractor *hae) 
	: bs(bs), haextractor(hae), use_min_cost(false) 
{
  	heuristic = new RelaxedPlanHeuristic(*bs);
	/*
    	for(unsigned i = 2; i < task.useful_ops().size(); i++) 
	{
		if(task.useful_ops()[i]->metric_cost() == 0.0) 
		{
			use_min_cost = true;
			break;
		}
	}
	*/
}

HARPHeuristicAdapter::~HARPHeuristicAdapter()
{
   	delete heuristic;
     	delete bs;
       	delete haextractor;
}



float HARPHeuristicAdapter::eval( State &s ) 
{
    	std::vector<unsigned> dummy;
	return eval(s, dummy);
}

float HARPHeuristicAdapter::eval( State &s, std::vector<unsigned> &helpful ) 
{
	m_relaxed_plan.clear(); 
	heuristic->eval(s, m_relaxed_plan);

	if ( m_relaxed_plan.cost() == std::numeric_limits<float>::max() )
		return m_relaxed_plan.cost();       
 
	int num_actions = m_relaxed_plan.size();
	float cost = (m_relaxed_plan.cost() + (use_min_cost ? (num_actions * min_cost) : 0.0));

	m_num_obs_accounted = 0;
	for ( RelaxedPlan::iterator it = m_relaxed_plan.begin();
		it != m_relaxed_plan.end(); it++ )
		if ( task.is_obs( *it ) || task.is_forgo( *it ) )
			m_num_obs_accounted++;   
	
	haextractor->extract(s, m_relaxed_plan, helpful);
       
	return cost;
}

float HARPHeuristicAdapter::eval( State &s, std::vector<unsigned> &helpful, std::vector<unsigned>& partial_plan  ) 
{
  
	eval_count++;

	m_relaxed_plan.clear();
	m_relaxed_plan.validate(); 
	heuristic->eval(s, m_relaxed_plan, partial_plan);

	/*
	#ifndef NDEBUG
	std::cout << "Relaxed plan:" << std::endl;
	m_relaxed_plan.print_layered(s, std::cout);
	std::cout << std::endl;
	std::cout << (m_relaxed_plan.valid() ? "Which is valid" : "Which is invalid" ) << std::endl;
	#endif
	*/

	if ( m_relaxed_plan.cost() == std::numeric_limits<float>::max() )
		return m_relaxed_plan.cost();       
	int num_actions = m_relaxed_plan.size();
	float cost = (m_relaxed_plan.cost() + (use_min_cost ? (num_actions * min_cost) : 0.0));

	m_num_obs_accounted = 0;
	for ( RelaxedPlan::iterator it = m_relaxed_plan.begin();
		it != m_relaxed_plan.end(); it++ )
		if ( task.is_obs( *it ) || task.is_forgo( *it ) )
			//if ( s.can_apply(*it) || s.atom_set().contains( task.useful_ops()[*it]->preconds() ) )
				m_num_obs_accounted++;   
	
	haextractor->extract(s, m_relaxed_plan, helpful);

	return cost;
}

}
