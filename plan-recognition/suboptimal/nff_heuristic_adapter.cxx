#include "nff_heuristic_adapter.hxx"

namespace NFF
{

HeuristicAdapter::HeuristicAdapter()
	: task(PDDL::Task::instance()), eval_count(0) 
{ 
}

HeuristicAdapter::~HeuristicAdapter()
{
}

}
