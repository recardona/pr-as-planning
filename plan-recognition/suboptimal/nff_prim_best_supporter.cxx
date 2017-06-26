#include "nff_prim_best_supporter.hxx"

namespace NFF
{

void PrimBestSupporter::initialize( State &s ) 
{
    	BestSupporter::initialize(s);
	PrimActionOrdering::setState(s);
	while(! actions_queue.empty() ) 
	{
		actions_queue.pop();
	}
	for(unsigned i = 0; i < to_apply[0].size(); i++) 
	{
		actions_queue.push(to_apply[0][i]);
	}
	num_to_support = task.fluents().size() - s.atom_vec().size();
}

void PrimBestSupporter::computeSupports( State &s ) 
{
    	initialize(s);
	doPropagation();
	//    printSupports();
}

PDDL::Task& PrimActionOrdering::task = PDDL::Task::instance();
std::vector<unsigned> PrimActionOrdering::uf_PCs;

void PrimBestSupporter::doPropagation() 
{

    	while(num_to_support && !actions_queue.empty() ) 
	{

		unsigned act = actions_queue.top();
		actions_queue.pop();
		Atom_Vec &adds = task.useful_ops()[act]->add_vec();
		for(unsigned i = 0; i < adds.size(); i++) 
		{
			if(support[adds[i]] == task.useful_ops().size()) 
			{
				setSupporter(adds[i], act);
				num_to_support--;
				std::vector<unsigned> &req_by = task.required_by(adds[i]);  
				for(unsigned j = 0; j < req_by.size(); j++) 
				{
					unsigned trigger_action = req_by[j];
					uf_pc_count[trigger_action]--;
					if(uf_pc_count[trigger_action] == 0) 
					{
						actions_queue.push(trigger_action);
					}
				}
			}
		}
	}
}

}
