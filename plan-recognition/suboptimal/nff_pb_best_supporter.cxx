#include "nff_pb_best_supporter.hxx"
#include <limits>

namespace NFF
{

PBBestSupporter::PBBestSupporter()
{
	fact_cost.resize(task.fluents().size());
	pcs_cost.resize(task.useful_ops().size());
	//      aa.resize(task.useful_ops().size());
}

PBBestSupporter::~PBBestSupporter()
{
}

void PBBestSupporter::printSupport(unsigned f, std::ostream& os) 
{
    	BestSupporter::printSupport(f, os);
	os << "(fluent cost: " << getFluentCost(f) << ")" << std::endl;
}

void PBBestSupporter::initialize( State &s ) 
{

    	BestSupporter::initialize(s);

	for( unsigned i = 0; i < task.useful_ops().size(); i++ ) 
	{
		pcs_cost[i] = std::numeric_limits<float>::infinity();
	}

	for(unsigned i = 1; i < task.fluents().size(); i++) 
	{
		fact_cost[i] = std::numeric_limits<float>::infinity();
	}
	
	for(unsigned i = 0; i < s.atom_vec().size(); i++) 
	{
		fact_cost[s.atom_vec()[i]] = 0.0;
	}
}


}
