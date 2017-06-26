#include "nff_best_supporter.hxx"
#include "nff_options.hxx"
#include <cmath>
#include <algorithm>

namespace NFF
{

BestSupporter::BestSupporter()
	: task( PDDL::Task::instance() )
{
	cost_sort_done = false;
	support.resize(task.fluents().size());
	//      fact_level.resize(task.fluents().size());
	//      action_level.resize(task.useful_ops().size());
	uf_pc_count.resize(task.useful_ops().size());
	to_apply.resize(1);
}

BestSupporter::~BestSupporter()
{
}
   
void BestSupporter::initialize(State &s) 
{

    	to_apply[0].clear();

	for(unsigned i = 1; i < task.useful_ops().size(); i++) 
	{
		PDDL::Operator* op_ptr = task.useful_ops()[i];
		uf_pc_count[i] = op_ptr->prec_vec().size();
		if ( op_ptr->accounts_obs() && uf_pc_count[i] > 0 )					uf_pc_count[i]--;
		if(0 == uf_pc_count[i]) 
		{
			to_apply[0].push_back(i);
		}
	}

	for(unsigned i = 1; i < task.fluents().size(); i++) 
		support[i] = task.useful_ops().size();

	for(unsigned i = 0; i < s.atom_vec().size(); i++) 
	{
		unsigned ft = s.atom_vec()[i];
		support[ft] = 0;
		for(unsigned j = 0; j < task.required_by(ft).size(); j++) 
		{
			uf_pc_count[task.required_by(ft)[j]]--;
			if(0 == uf_pc_count[task.required_by(ft)[j]]) 
			{
				to_apply[0].push_back(task.required_by(ft)[j]);
			}
		}
	}
}

void BestSupporter::extractPlan( State& s, RelaxedPlan& rp )
{
	Operator_Queue		need_processing;
	Bool_Vec		seen( task.useful_ops().size() );

	need_processing.push( task.end() );
	seen[task.end()] = true;
	/*
	#ifndef NDEBUG
	std::cout << "Extracting relaxed plan:" << std::endl;
	#endif
	*/
	unsigned count_ops = 1;
	while (!need_processing.empty() )
	{
		unsigned op = need_processing.front();
		need_processing.pop();
		rp.insert(op);
		PDDL::Operator* op_ptr = task.useful_ops()[op];
		/*
		#ifndef NDEBUG
		std::cout << "Processing operator # "<<  count_ops++ <<" : ";
		task.print_operator( op, std::cout );
		std::cout << std::endl;
		#endif
		*/
		// select supporters for operator preconditions
		Atom_Vec& op_precs = op_ptr->prec_vec();
		for ( unsigned k = 0; k < op_precs.size(); k++ )
		{
			unsigned pk = op_precs[k];
			PDDL::Fluent* pk_ptr = task.fluents()[pk];
			if ( !pk_ptr->enabled() ) 
			{
				/*
				#ifndef NDEBUG
				std::cout << "\t Processing precondition: ";
				task.print_fluent( pk, std::cout );
				std::cout << " ";

				std::cout << "which is disabled, so it gets ignored" << std::endl;
				#endif
				*/
				continue; // disabled fluents are ignored
			}
			if ( supporter(pk) == task.useful_ops().size() )
			{
				/*
				#ifndef NDEBUG
				std::cout << "\t Processing precondition: ";
				task.print_fluent( pk, std::cout );
				std::cout << " ";

				std::cout << "which lacks a valid supporter, so the relaxed plan is invalid as well" << std::endl;
				#endif
				*/
				// No valid supporter in the heuristic
				rp.invalidate();
				return;
			}
			if ( supporter(pk) == task.start() )
			{
				/*
				#ifndef NDEBUG
				std::cout << "\t Processing precondition: ";
				task.print_fluent( pk, std::cout );
				std::cout << " ";

				std::cout << "which is already true in the state s";
				std::cout << std::endl;
				#endif
				*/
			}
			else
			{
				if ( !seen[supporter(pk)] ) 
				{
					/*
					#ifndef NDEBUG	
					std::cout << "\t Processing precondition: ";
					task.print_fluent( pk, std::cout );
					std::cout << " ";

					std::cout << "which is supported by operator ";
					task.print_operator( supporter(pk), std::cout );
					std::cout << std::endl;
					#endif
					*/
					need_processing.push( supporter(pk) );
					seen[supporter(pk)] = true;
				}
				else
				{
					/*
					#ifndef NDEBUG
					std::cout << "\t Processing precondition: ";
					task.print_fluent( pk, std::cout );
					std::cout << " ";

					std::cout << "which is supported by operator ";
					task.print_operator( supporter(pk), std::cout );
					std::cout << ", an operator which has been seen already";
					std::cout << std::endl;
					#endif
					*/
				}
			}
		}
	}	
}

void BestSupporter::extractPlan( RelaxedPlan &rp) 
{
    	//extractPlanForAction(task.end(), rp);
    	rp.insert( task.end() );
    	PDDL::Operator* end_op = task.useful_ops()[task.end()];
	for ( unsigned k = 0; k < end_op->prec_vec().size(); k++ )
	{
		unsigned gk = end_op->prec_vec()[k];
		PDDL::Fluent* gk_ptr = task.fluents()[gk];
		if ( !gk_ptr->enabled() ) continue;
		if ( supporter(gk) == task.useful_ops().size() )
		{
			std::cout << "WHOOPS: Atom ";
			task.print_fluent(gk, std::cout );
			std::cout << " has no defined supporter!!!" << std::endl;
			rp.invalidate();
		}
		else
			extractPlanForAction( supporter(gk), rp );
	}
}

void BestSupporter::extractPlanAssuming( RelaxedPlan &rp, unsigned assume) 
{
    	extractPlanForActionAssuming(task.end(), rp, assume);
}
  
void BestSupporter::extractPlanForAction(unsigned action,
				      RelaxedPlan &rp) 
{
	NFF_Options& opt = NFF_Options::instance();
	#ifndef NDEBUG
	if ( opt.verbose_mode() > 2 )
	{
		std::cout << "Extracting plan for action ";
		task.print_operator(action,std::cout);
		std::cout << " (" << action << ")" << std::endl;
	}
	#endif
	
	if(action == 0) return;
	if( rp.find( action ) != rp.end() ) return;
	if ( action == task.useful_ops().size() )
	{
		std::cout << "WHOOPS" << std::endl;
		rp.invalidate();
		return;
	}
	rp.insert(action);
	for(unsigned i = 0; i < task.useful_ops()[action]->prec_vec().size(); i++) 
	{
		unsigned prec_i =  task.useful_ops()[action]->prec_vec()[i];
		PDDL::Fluent*  prec_i_ptr = task.fluents()[prec_i];
		if ( !prec_i_ptr->enabled() ) continue;
		if ( supporter(prec_i) == task.useful_ops().size() )
		{
			std::cout << "WHOOPS: Atom ";
			task.print_fluent(prec_i, std::cout );
			std::cout << " has no defined supporter!!!" << std::endl;
			rp.invalidate();
		}
		else
			extractPlanForAction(supporter(prec_i), rp);
	}    
}

void BestSupporter::extractPlanForActionAssuming(unsigned action,
					      RelaxedPlan &rp, 
					      unsigned assume) 
{
    	/*
	std::cout << "Extracting plan for action ";
	task.print_operator(action);
	std::cout << std::endl; */
	
	if(action == 0) return;
	rp.insert(action);
	for(unsigned i = 0; i < task.useful_ops()[action]->prec_vec().size(); i++) 
	{
		/*
		std::cout << "Precondition " << i << ": ";
		printSupport(task.useful_ops()[action]->prec_vec()[i]);*/
		if(task.useful_ops()[action]->prec_vec()[i] != assume)
			extractPlanForActionAssuming(supporter(task.useful_ops()[action]->prec_vec()[i]), rp, assume);
	}
}

void BestSupporter::extractPlanForFluent(unsigned fluent, RelaxedPlan &rp) 
{
    	//std::cout << "extractPlanForFluent(" << fluent << ")" << std::endl;
	extractPlanForAction(supporter(fluent), rp);
}

void BestSupporter::extractPlanForFluentAssuming(unsigned fluent, RelaxedPlan &rp, unsigned assuming) 
{
    	extractPlanForActionAssuming(supporter(fluent), rp, assuming);
}

void BestSupporter::printSupports( ) 
{
    	printSupports(std::cout);
}

void BestSupporter::printSupports(std::ostream& os ) 
{
    	os << "Supports:" << std::endl;
	for(unsigned i = 1; i < task.fluents().size(); i++) 
	{
		printSupport(i, os);
		os << std::endl;
	}
}

void BestSupporter::printSupport(unsigned f) 
{
    	printSupport(f, std::cout);
}
  
void BestSupporter::printSupport(unsigned f, std::ostream& os) 
{
    	task.print_fluent(f, os);
	os << "(" << f << ")";
	
	os << " --> ";
	if(support[f] == task.useful_ops().size()) 
	{
		os << "UNSUPPORTED";
	}
	else 
	{
		task.print_operator(support[f], os);
	}
}

void BestSupporter::getDependencies(Atom_Vec& pcs, std::set<unsigned> &deps) 
{
    	for(unsigned i = 0; i < pcs.size(); i++) 
	{
		getDependencies(pcs[i], deps);
	}
}

void BestSupporter::getDependencies(unsigned pc, std::set<unsigned> &deps) 
{
    	deps.insert(pc);
	if(support[pc] == 0) return;
	getDependencies(task.useful_ops()[support[pc]]->prec_vec(), deps);
}

std::vector<unsigned> & BestSupporter::getCostSortedOpVec() 
{
	if(! cost_sort_done) 
	{
		for(unsigned i = 1; i < task.useful_ops().size(); i++)
			cost_sorted_actions.push_back(i);
		std::sort(cost_sorted_actions.begin(), cost_sorted_actions.end(), increasingActionCostSort(*this));
		cost_sort_done = true;
	}
	return cost_sorted_actions;
}  

}
