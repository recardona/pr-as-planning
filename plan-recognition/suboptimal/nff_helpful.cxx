#include "nff_state.hxx"
#include "nff_helpful.hxx"
#include "nff_relaxed_plan.hxx"
#include "nff_best_supporter.hxx"
#include "PDDL.hxx"
#include <algorithm>

namespace NFF {

void PlanActions::extract(State &s, RelaxedPlan &rp, std::vector<unsigned> &helpful ) 
{
    
    	std::set<unsigned>::iterator it;

	for(it = rp.begin(); it != rp.end(); it++) 
	{
		if(s.can_apply(*it)) 
		{
			helpful.push_back(*it);
		}
	}
}

void NoActions::extract( State &s, RelaxedPlan &rp, std::vector<unsigned> &helpful ) 
{
    	// just so we don't do any work here.
	return;
}

void RPHA_w_mutexes::extract( State &s, RelaxedPlan &rp, std::vector<unsigned> &helpful ) 
{

    	static std::vector<std::vector<unsigned> > is_pc_for(task.fluents().size());
	static std::vector<unsigned> other_precs, added_precs;
	static RelaxedPlan rp_local;
	std::set<unsigned>::iterator it;
	
	helpful.clear();
	for(unsigned i = 0; i < task.fluents().size(); i++) 
	{
		is_pc_for[i].clear();
	}

	for(it = rp.begin(); it != rp.end(); it++) 
	{
		PDDL::Operator* rp_op_ptr = task.useful_ops()[*it];
		Atom_Vec& precs = rp_op_ptr->prec_vec();
		for(unsigned i = 0; i < precs.size(); i++) 
		{
			// pc_of_rp[task.useful_ops()[*it]->prec_vec()[i]] = true;
			// for each pc, make a list of plan ops for which it is a pc
			unsigned prec_i = precs[i];
			PDDL::Fluent* prec_i_ptr = task.fluents()[prec_i];
			if ( prec_i_ptr->enabled() )
				is_pc_for[prec_i].push_back(*it);
		}
	}
	for(unsigned op = 1; op < task.useful_ops().size(); op++) 
	{
		if( s.can_apply(op) ) 
		{
			if ( task.is_obs(op) && rp.find(op) == rp.end() ) continue;
			// check if at least one pc of rp is added
			/*
			#ifndef NDEBUG
			std::cout << "Considering ";
			task.print_operator(op, std::cout);
			std::cout << std::endl;
			#endif
			*/
			PDDL::Operator* op_ptr = task.useful_ops()[op];
			Atom_Vec& op_adds = op_ptr->add_vec();
			unsigned i;
			// for each added fluent
			for(i = 0; i < op_adds.size(); i++) 
			{
				unsigned add_i = op_adds[i];
				PDDL::Fluent* ptr_add_i = task.fluents()[op_adds[i]];
				if ( !ptr_add_i->enabled() ) continue;
				/*
				#ifndef NDEBUG
				std::cout << " as support for atom ";
				task.print_fluent(  add_i, std::cout );
				std::cout << " which is a precondition of" << std::endl;
				#endif
				*/
				// for each action for which the fluent is a pc
				std::vector<unsigned> &actions_fw_pc = is_pc_for[add_i];
				for(unsigned j = 0; j < actions_fw_pc.size(); j++) 
				{
					/*
					#ifndef NDEBUG
					std::cout << "\t";
					task.print_operator(actions_fw_pc[j], std::cout);
					std::cout << std::endl;
					#endif
					*/
					// check whether it is valid support for actions_fw_pc[j]
					Atom_Vec &pcs = task.useful_ops()[actions_fw_pc[j]]->prec_vec();
		
					unsigned k;
					for(k = 0; k < pcs.size(); k++) 
					{
						if ( !task.fluents()[pcs[k]]->enabled() ) continue;
						if( !op_ptr->adds().isset(pcs[k]) ) 
						{

							// for each of the pcs not added by the possible support
							// get the local plan
							rp_local.clear();
							bs.extractPlanForFluent(pcs[k], rp_local);
							/* if op edeletes one of the pcs of the action 
							its supposed to support */
							if( (rp_local.begin() == rp_local.end() )
								&& (task.fast_op_edeletes(op).isset(pcs[k]) || op_ptr->dels().isset(pcs[k])) ) 
							{
								/*
								#ifndef NDEBUG
								std::cout << "\tEmpty plan, and op edeletes ";
								task.print_fluent(pcs[k], std::cout);
								std::cout << std::endl;
								#endif
								*/
								break;
							}
		
							// check whether the local plan deletes the pcs added by op
							for(it = rp_local.begin(); it != rp_local.end(); it++) 
							{
								unsigned m;
								for(m = 0; m < pcs.size(); m++) 
								{
									if ( !task.fluents()[pcs[m]]->enabled() ) continue;
									// for the pcs added by the candidate ha, check that they are not 
									// deleted by the rp for the other pcs.
									if(op_ptr->adds().isset(pcs[m])) 
									{
										PDDL::Operator* op_it = task.useful_ops()[*it]; 
										if(task.fast_op_edeletes(*it).isset(pcs[m])
											|| op_it->dels().isset(pcs[m])
										) 
										{
											// op is an invalid support for actions_fw_pc[j]
											/*
											#ifndef NDEBUG
											std::cout << "\tPlan operator ";
											task.print_operator(*it, std::cout);
											std::cout << " edeletes ";
											task.print_fluent(pcs[m], std::cout);
											std::cout << std::endl;
											#endif
											*/
											break;
										}
									}
								}
								if(m != pcs.size()) 
								{
									break;
								}
							}
							if(it != rp_local.end()) 
							{
								break;
							}
						}
					}

					if(k == pcs.size()) 
					{
						// valid support for at least one action
						if ( std::find( helpful.begin(), helpful.end(), op ) == helpful.end() )
							helpful.push_back(op);
						/*
						#ifndef NDEBUG	
						task.print_operator(op, std::cout);
						std::cout << " chosen as valid support for ";
						task.print_operator(actions_fw_pc[j], std::cout);
						std::cout << std::endl;
						#endif
						*/
					  	// so skip looking at other action fw it might be a support
					  	break;
					}
				}
			}
		}
	}
}

void HelpfulActionExtractor::extract( State &s, RelaxedPlan &rp, std::vector<unsigned> &helpful ) 
{

    	helpful.clear();

	static std::vector<bool> pc_of_rp(task.fluents().size(), false);

	pc_of_rp.assign(task.fluents().size(), false);

	std::set<unsigned>::iterator it;
	for(it = rp.begin(); it != rp.end(); it++) 
	{
		PDDL::Operator *op = task.useful_ops()[*it];
		for(unsigned i = 0; i < op->prec_vec().size(); i++) 
		{
			pc_of_rp[op->prec_vec()[i]] = true;
		}
	}
	for(unsigned op = 1; op < task.useful_ops().size(); op++) 
	{
		if( s.can_apply(op) ) 
		{
			unsigned i;
			for(i = 0; i < task.useful_ops()[op]->add_vec().size(); i++) 
			{
				if(pc_of_rp[task.useful_ops()[op]->add_vec()[i]])
					break;
			}
			if(i != task.useful_ops()[op]->add_vec().size()) 
			{
				helpful.push_back(op);
			}
		}
	}
}

}
