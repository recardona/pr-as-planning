#include "nff_hadd_best_supporter.hxx"
#include "ext_math.hxx"

namespace NFF
{

void haddBestSupporter::computeSupports( State &s ) 
{
    	haddBestSupporter::initialize( s );
	haddBestSupporter::doPropagation( );
	//    printSupports();
}

void haddBestSupporter::doPropagation( ) 
{

    	unsigned lev = 0, act;
	static std::vector<unsigned>::iterator it;
	float act_cost;
	
	while( ! to_apply[lev].empty() ) 
	{
	
		if((lev + 1) == to_apply.size()) 
		{
			to_apply.resize(to_apply.size() + 1);
		}
		to_apply[lev + 1].clear();

		for(it = to_apply[lev].begin(); it != to_apply[lev].end(); it++)  
		{
			act = *it;
			act_cost = 0.0;
			for(unsigned i = 0; i < task.useful_ops()[act]->prec_vec().size(); i++) 
			{
				act_cost += getFluentCost(task.useful_ops()[act]->prec_vec()[i]);
			}
			/*
			std::cout << "act_cost for ";
			task.print_operator(act);
			std::cout << " (" << act << ")";
			std::cout << "is " << act_cost;
			std::cout << ", previously " << getPCsCost(act) << std::endl;*/
	
			if(dless(act_cost, getPCsCost(act))) 
			{
				//	  action_level[act] = lev;
				pcs_cost[act] = act_cost;
				act_cost += getActionCost(act);
				Atom_Vec &adds = task.useful_ops()[act]->add_vec();
				for(unsigned i = 0; i < adds.size(); i++) 
				{
					std::vector<unsigned> &req_by = task.required_by(adds[i]);
					float fc = getFluentCost(adds[i]);
					// fluent achieved the first time
					if(fc == std::numeric_limits<float>::infinity()) 
					{
						setSupporter(adds[i], act);
						for(unsigned j = 0; j < req_by.size(); j++) 
						{
							unsigned trigger_action = req_by[j];
							uf_pc_count[trigger_action]--;
							if(uf_pc_count[trigger_action] == 0) 
							{
								to_apply[lev + 1].push_back(trigger_action);
							}
						}
					}
					// fluent achieved previously, cost decrease
					else if(dless(act_cost, fc)) 
					{
						setSupporter(adds[i], act);
						for(unsigned j = 0; j < req_by.size(); j++) 
						{
							if(uf_pc_count[req_by[j]] == 0) 
							{
								to_apply[lev + 1].push_back(req_by[j]);
							}
						}
					}
				}
			}
		}
		lev++;
	}
}


}
