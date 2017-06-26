#ifndef __NFF_HEURISTIC_ADAPTER__
#define __NFF_HEURISTIC_ADAPTER__

#include "nff_heuristic.hxx"

namespace NFF
{

class HeuristicAdapter 
{

protected:
    	PDDL::Task &task;
	unsigned eval_count;
    
public:
	HeuristicAdapter();
    	virtual ~HeuristicAdapter();
	virtual bool suggestsHelpful() { return false; }
	virtual float eval( State &s ) 
	{
		std::vector<unsigned> dummy;
		return eval(s, dummy);
	}
	virtual float eval( State &s, std::vector<unsigned> &helpful ) = 0;

	virtual unsigned getEvalCount() { return eval_count; }
};

}

#endif // nff_heuristic_adapter.hxx
