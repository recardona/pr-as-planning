#ifndef _NFF_RELAXED_PLAN__
#define _NFF_RELAXED_PLAN__

#include "PDDL.hxx"
#include "nff.hxx"
#include "nff_state.hxx"

namespace NFF
{

class RelaxedPlan : public std::set<unsigned> 
{

public:
	RelaxedPlan();
	virtual ~RelaxedPlan();
	inline PDDL::Task& task() { return PDDL::Task::instance(); }
	float cost();
	void print( std::ostream& os );
	void print() { print(std::cout); }
	void print_layered(State &s, std::ostream& os );
	
	inline void print_layered(State &s ) { print_layered(s, std::cout); }
	void linearize( State& s, Operator_Vec& linear_plan );
	void invalidate() { m_valid = false; }
	bool valid() { return m_valid; }
	void validate() { m_valid = true; }
	PDDL::Fluent_Set*	required_atoms();
	PDDL::Fluent_Set*	added_atoms();

protected:
	bool m_valid;
};

}

#endif // nff_relaxed_plan.hxx
