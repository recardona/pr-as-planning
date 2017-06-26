#ifndef __NFF_PRIM_BEST_SUPPORTER__
#define __NFF_PRIM_BEST_SUPPORTER__

#include "nff_best_supporter.hxx"
#include <queue>

namespace NFF
{

class PrimActionOrdering 
{
protected:
    	static PDDL::Task& task;
	static std::vector<unsigned> uf_PCs;
public:
	PrimActionOrdering() 
	{
		uf_PCs.resize(task.useful_ops().size());
	}
    
	bool operator() (const unsigned &a1, const unsigned &a2) const 
	{
		return ((task.useful_ops()[a1]->metric_cost() > task.useful_ops()[a2]->metric_cost()) ||
			((task.useful_ops()[a1]->metric_cost() == task.useful_ops()[a2]->metric_cost()) && 
			(uf_PCs[a1] > uf_PCs[a2])));
	}
    
	static void setState(State &s) 
	{
		for(unsigned i = 0; i < task.useful_ops().size(); i++) 
		{
			uf_PCs[i] = 0;
			for(unsigned j = 0; j < task.useful_ops()[i]->prec_vec().size(); j++) 
			{
				if(! s.atom_set().isset(task.useful_ops()[i]->prec_vec()[j])) 
				{
					uf_PCs[i]++;
				}
			}
		}
	}
};
  
class PrimBestSupporter : public BestSupporter 
{

protected:
    	virtual void doPropagation();
	virtual void initialize(State &s);

	unsigned num_to_support;

	friend class PrimActionOrdering;
	std::priority_queue<unsigned, std::vector<unsigned>, PrimActionOrdering> actions_queue;
    
public:
	PrimBestSupporter() { }
	void computeSupports(State &s);
};

}

#endif // nff_prim_best_supporter.hxx
