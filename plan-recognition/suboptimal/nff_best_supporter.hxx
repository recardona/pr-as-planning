#ifndef __NFF_BEST_SUPPORTER__
#define __NFF_BEST_SUPPORTER__

#include <vector>
#include <set>
#include "nff.hxx"
#include "nff_state.hxx"
#include "PDDL.hxx"
#include "nff_relaxed_plan.hxx"

namespace NFF
{

class BestSupporter
{  
protected:
   
    	std::vector<unsigned> support;
	//    std::vector<unsigned> fact_level;
	//    std::vector<unsigned> action_level;
	
	std::vector<unsigned> uf_pc_count;
	
	std::vector<std::vector<unsigned> > to_apply;
	std::vector<unsigned> part_plan;
	
	PDDL::Task &task;
	
	// whether vector of actions sorted by cost has been constructed.
	bool cost_sort_done;
	std::vector<unsigned> cost_sorted_actions;
	
	void getDependencies(Atom_Vec& pcs, std::set<unsigned> &deps);
	void getDependencies(unsigned atm, std::set<unsigned> &deps);
	
	virtual void initialize( State &s );
	
	virtual void setSupporter(unsigned f, unsigned a) 
	{
		support[f] = a;
		//      fact_level[f] = getActionLevel(a) + 1;
	}

	unsigned count_supported_atoms( unsigned a )
	{
		unsigned count = 0;
		for ( unsigned k = 0; k < support.size(); k++ )
			if ( support[k] == a ) count++;
		return count;
	}
	
	class increasingActionCostSort 
	{
	protected:
		BestSupporter &bs;
	public:
		increasingActionCostSort(BestSupporter &bs) : bs(bs) {}
		bool operator() (const int &a1, const int &a2) const 
		{
			return (bs.getActionCost(a1) < bs.getActionCost(a2));
		}
	};    

public:
  
	BestSupporter();
	virtual ~BestSupporter();
  
	virtual unsigned& supporter( unsigned fluent ) { return support[fluent]; }
	virtual void computeSupports( State &s ) = 0;
	virtual void computeSupports( State &s, std::vector<unsigned>& p_plan )
	{
		part_plan.clear();
		part_plan.assign( p_plan.begin(), p_plan.end() );
		computeSupports(s);
	}
	
	virtual void printSupports( std::ostream& os );
	virtual void printSupports();
	
	virtual void printSupport(unsigned f, std::ostream& os );
	virtual void printSupport(unsigned f);
	
	virtual float getActionCost(unsigned i) 
	{ 
		return task.useful_ops()[i]->metric_cost(); 
	}
	
	//    virtual unsigned getFactLevel(unsigned f) { return fact_level[f]; }
	//    virtual unsigned getActionLevel(unsigned a) { return action_level[a]; }


	void extractPlan( State& s, RelaxedPlan &rp);
	void extractPlan( RelaxedPlan &rp);
	void extractPlanForAction(unsigned action, RelaxedPlan &rp);
	void extractPlanForFluent(unsigned fluent, RelaxedPlan &rp);

	void extractPlanAssuming( RelaxedPlan &rp, unsigned assume);
	void extractPlanForActionAssuming(unsigned action, RelaxedPlan &rp, unsigned assume);
	void extractPlanForFluentAssuming(unsigned fluent, RelaxedPlan &rp, unsigned assume);

	virtual std::vector<unsigned> &getCostSortedOpVec();
	std::vector<unsigned>&		partial_plan() { return part_plan; }
};


}

#endif // nff_best_supporter
