#include "nff_hsa_best_supporter.hxx"
#include "ext_math.hxx"
#include "nff_options.hxx"

namespace NFF
{

hsaBestSupporter::hsaBestSupporter()
{
	supportPlan.resize(task.fluents().size());
}

hsaBestSupporter::~hsaBestSupporter()
{
}

void hsaBestSupporter::printSupport(unsigned f, std::ostream& os) 
{
    	PBBestSupporter::printSupport(f, os) ;
	os << "\thsa plan: ";
	((RelaxedPlan)(supportPlan[f])).print(os);
}

void hsaBestSupporter::printSupport(unsigned f) 
{
    	printSupport(f, std::cout);
}

void   hsaBestSupporter::initialize( State &s ) 
{
	PBBestSupporter::initialize(s);
	
	for(unsigned i = 1; i < task.fluents().size(); i++) 
		supportPlan[i].clear();
}

void hsaBestSupporter::computeSupports( State &s ) 
{
    	hsaBestSupporter::initialize( s );
	current_state = &s;
	hsaBestSupporter::doPropagation( );
}

// add l2 to l1
void hsaBestSupporter::merge(RelaxedPlan &l1, RelaxedPlan &l2) 
{
    
	static RelaxedPlan::iterator it1, it2;

	it1 = l1.begin();
	it2 = l2.begin();
    
	while((it1 != l1.end()) && (it2 != l2.end())) 
	{
		// do sorted insertions
		if(*it1 == *it2) 
		{
			it1++;
			it2++;
		}
		else if(*it1 > *it2) 
		{
			l1.insert(it1, *it2);
			it2++;
		}
		else 
		{
			it1++;
		}
	}
	// insert at end
	while(it2 != l2.end()) 
	{
		l1.insert(it1, *it2);
		it1++;
		it2++;
	}
}

bool hsaBestSupporter::valid_obs_extension( unsigned ao, RelaxedPlan& rp )
{
	if ( !task.is_obs( ao ) ) return true;

	// Plan Prefix Test
	for ( std::vector<unsigned>::reverse_iterator it = partial_plan().rbegin();
		it != partial_plan().rend(); it++ )
	{
		if ( *it == ao ) return false; // already in plan
		if ( task.is_obs(*it) && task.precedes_in_obs_seq( ao, *it ) )
			return false;
	}	

	// Relaxed Plan Test
	for ( RelaxedPlan::iterator it2 = rp.begin(); it2 != rp.end(); it2++ )
		if ( task.is_obs(*it2) && task.precedes_in_obs_seq( ao, *it2 ) )
			return false;

	return true;
} 

bool hsaBestSupporter::check_for_loops( Operator_Vec& supportees, unsigned prec, RelaxedPlan& pi )
{
	for ( unsigned k = 0; k < supportees.size(); k++ )
		if ( pi.find( supportees[k] ) != pi.end() ) 
		{
			/*
			#ifndef NDEBUG
			std::cout << "Loop Found: ";
			std::cout << "Operator: ";
			task.print_operator( supportees[k], std::cout );
			std::cout << std::endl;
			std::cout << "Found in relaxed plan for one of its preconditions: " << std::endl; 
			std::cout << "Precondition: ";
			task.print_fluent( prec, std::cout );
			std::cout << std::endl;
			std::cout << "Relaxed plan: ";
			pi.print_layered( *current_state, std::cout );
			std::cout << std::endl;
			#endif
			*/
			return true;
		}
	return false;
}
 
void hsaBestSupporter::doPropagation( ) 
{
     	static RelaxedPlan rp;
	static std::vector<unsigned>::iterator it;
	static RelaxedPlan::iterator it2;

	unsigned lev = 0;

	while( ! to_apply[lev].empty() ) 
	{

		if((lev + 1) == to_apply.size()) 
		{
			to_apply.resize(to_apply.size() + 1);
		}
		to_apply[lev + 1].clear();

		for(it = to_apply[lev].begin(); it != to_apply[lev].end(); it++)  
		{
			unsigned act = *it;
			PDDL::Operator* act_ptr = task.useful_ops()[act];
			rp.clear();
			for(unsigned i = 0; i < act_ptr->prec_vec().size(); i++) 
			{
				unsigned prec_i = act_ptr->prec_vec()[i];
				PDDL::Fluent* prec_i_ptr = task.fluents()[prec_i];
				if( prec_i_ptr->enabled() && !current_state->atom_set().isset( prec_i) )//getFluentCost(prec_i) != 0.0) 
				{
					merge(rp, supportPlan[prec_i]);
				}
			}
			if ( act_ptr->accounts_obs() && current_state->atom_set().isset( act_ptr->explains() ) )
				continue;

			float act_cost = 0.0;
			for(it2 = rp.begin(); it2 != rp.end(); it2++) 
			{
				act_cost += getActionCost(*it2);
			}
	
			if(dless(act_cost, getPCsCost(act))) 
			{
				//#ifndef NDEBUG
				//task.print_operator( act, std::cout );
				//std::cout << std::endl;
				//#endif 
				pcs_cost[act] = act_cost;
				act_cost += getActionCost(act);
	
				//it2 = rp.begin();
				//while((*it2 < act) && (it2 != rp.end())) it2++;
				rp.insert(it2, act);
	
				Atom_Vec &adds = act_ptr->add_vec();
				for(unsigned i = 0; i < adds.size(); i++) 
				{
					if ( !task.fluents()[adds[i]]->enabled() ) continue;
					if ( current_state->atom_set().isset( adds[i] ) ) continue;
					std::vector<unsigned> &req_by = task.required_by(adds[i]);
					// Loop checking
					if ( check_for_loops( req_by, adds[i], rp ) ) continue;
					//#ifndef NDEBUG
					//std::cout << "\t";
					//task.print_fluent( adds[i], std::cout );
					//#endif
					float fc = getFluentCost(adds[i]);
					// fluent achieved the first time
					if(fc == std::numeric_limits<float>::infinity()) 
					{
						//#ifndef NDEBUG
						//std::cout << " supported first time" << std::endl;
						//#endif
						setSupporter(adds[i], act, rp);
						for(unsigned j = 0; j < req_by.size(); j++) 
						{
							unsigned trigger_action = req_by[j];
							uf_pc_count[trigger_action]--;
							if(uf_pc_count[trigger_action] == 0) 
							{
								if ( valid_obs_extension( trigger_action, rp) )	
									to_apply[lev + 1].push_back(trigger_action);
							}
						}
					}
					// fluent achieved previously, cost decrease
					else if(dless(act_cost, fc)) 
					{
						//#ifndef NDEBUG
						//std::cout << " cost decreased, was " << fc << " will be " << act_cost << std::endl;
						//#endif
						setSupporter(adds[i], act, rp);
						for(unsigned j = 0; j < req_by.size(); j++) 
						{
							if(uf_pc_count[req_by[j]] == 0) 
							{
								if ( valid_obs_extension( req_by[j], rp) )	
									to_apply[lev + 1].push_back(req_by[j]);
							}
						}
					}
					else if ( dequal(act_cost,fc) )
					{
						// update if rp already associated to current fluent
						// has less explain ops inside
						//#ifndef NDEBUG
						//std::cout << " cost tied" << std::endl;
						//std::cout << "\t\tBreaking tie with obs" << std::endl;	
						//#endif
						unsigned num_exp_act = 0;
						for ( RelaxedPlan::iterator it = rp.begin();
							it != rp.end(); it++ )
							if ( task.is_obs( *it ) ) num_exp_act++;
						//#ifndef NDEBUG
						//std::cout << "\t\t\t new relaxed plan accounts for " << num_exp_act << " obs" << std::endl;
						//#endif
						unsigned num_exp_fluent = 0;
						for ( RelaxedPlan::iterator it = supportPlan[adds[i]].begin();
							it != supportPlan[adds[i]].end(); it++ )
							if ( task.is_obs( *it ) ) num_exp_fluent++;
						//#ifndef NDEBUG
						//std::cout << "\t\t\t new relaxed plan accounts for " << num_exp_fluent << " obs" << std::endl;
						//#endif
						if ( num_exp_act > num_exp_fluent )
						{
							setSupporter(adds[i], act, rp);
							for(unsigned j = 0; j < req_by.size(); j++) 
							{
								if(uf_pc_count[req_by[j]] == 0) 
								{
									if ( valid_obs_extension( req_by[j], rp) )	
										to_apply[lev + 1].push_back(req_by[j]);
								}
							}
						}
						if ( num_exp_act == num_exp_fluent )
						{
							if ( supportPlan[adds[i]].size() > rp.size() )
							{
								setSupporter(adds[i], act, rp);
								for(unsigned j = 0; j < req_by.size(); j++) 
								{
									if(uf_pc_count[req_by[j]] == 0) 
									{
										if ( valid_obs_extension( req_by[j], rp) )	
											to_apply[lev + 1].push_back(req_by[j]);
									}
								}
							}
						}
					}
					//#ifndef NDEBUG
					//else
					//{
					//	std::cout << "new cost: " << act_cost << " old cost: " << fc <<   std::endl;
					//}
					//#endif
				}
			}
		}
		lev++;
	}
}


}
