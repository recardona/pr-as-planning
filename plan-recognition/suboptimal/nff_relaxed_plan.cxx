#include "nff_relaxed_plan.hxx"
#include <cmath>
#include <limits>

namespace NFF
{

RelaxedPlan::RelaxedPlan()
	: m_valid( true )
{
}

RelaxedPlan::~RelaxedPlan()
{
}

float RelaxedPlan::cost() 
{
	if ( !valid() ) return std::numeric_limits<float>::max();
	float c = 0.0f;
	std::set<unsigned>::iterator it;
	for(it = begin(); it != end(); it++) 
	{
		c += task().useful_ops()[*it]->metric_cost();
	}
	return c;
}

void RelaxedPlan::print( std::ostream& os ) 
{
    	std::set<unsigned>::iterator it;
	os << "{";
	for(it = begin(); it != end();) 
	{
		task().print_operator(*it, os);
		os << "[" <<  task().useful_ops()[*it]->metric_cost() << "]";
		if(++it != end())
			os << ", ";
	}
	os << "}" << std::endl;
}

PDDL::Fluent_Set* RelaxedPlan::added_atoms()
{
	PDDL::Fluent_Set* eff = new PDDL::Fluent_Set( task().fluents().size() );
	std::set<unsigned>::iterator it;
	for ( it = begin(); it != end(); it++ )
	{
		PDDL::Operator* op_ptr = task().useful_ops()[*it];
		eff->add( op_ptr->adds() );
	}
	return eff;
}

PDDL::Fluent_Set* RelaxedPlan::required_atoms()
{
	PDDL::Fluent_Set* eff = new PDDL::Fluent_Set( task().fluents().size() );
	std::set<unsigned>::iterator it;
	for ( it = begin(); it != end(); it++ )
	{
		PDDL::Operator* op_ptr = task().useful_ops()[*it];
		eff->add( op_ptr->preconds() );
	}
	return eff;
}

void RelaxedPlan::linearize( State& s, Operator_Vec& linear_plan )
{
	PDDL::Fluent_Set fluents( s.atom_set() );

	for ( unsigned f = 1; f < task().fluents().size(); f++ )
		if ( task().is_explained(f) ) fluents.set(f);

	PDDL::Fluent_Set fluents_to_add( task().fluents().size() );
	
	std::set<unsigned> seen;
	std::set<unsigned>::iterator it;

	unsigned num_seen = 0, level = 0;
	
	bool fp = false;

	while ( ( num_seen < size() ) && !fp )
	{
		fp = true;
		level++;
		for ( it = begin(); it != end(); it++ )
		{
			if ( seen.find( *it ) == seen.end() )
			{
				if ( fluents.contains( task().useful_ops()[*it]->preconds() ) )
				{
					linear_plan.push_back( *it );
					num_seen++;
					fluents_to_add.add( task().useful_ops()[*it]->adds() );
					seen.insert( *it );
					fp = false;
				}
			}
		}
		fluents.add( fluents_to_add );
	}	

}

void RelaxedPlan::print_layered(State &s, std::ostream &os ) 
{

    	PDDL::Fluent_Set fluents( task().fluents().size() );
	for ( unsigned k = 0; k < s.atom_vec().size(); k++ )
		fluents.set( s.atom_vec()[k] );
	for ( unsigned f = 1; f < task().fluents().size(); f++ )
		if ( task().is_explained(f) ) fluents.set(f);

	PDDL::Fluent_Set fluents_to_add(task().fluents().size());
	std::set<unsigned> displayed;
	std::set<unsigned>::iterator it;
	
	unsigned num_displayed = 0;
	
	std::cout << "Relaxed Plan:" << std::endl;
	unsigned index = 0;
	std::cout << index << ":";
	bool fp = false;
	while((num_displayed < size()) && !fp) 
	{
		fp = true;
		for(it = begin(); it != end(); it++) 
		{
			if( displayed.find(*it) == displayed.end()) 
			{
				PDDL::Operator* op_ptr = task().useful_ops()[*it];
				bool can_apply = true;
				for ( unsigned k = 0; k < op_ptr->prec_vec().size(); k++ )
				{
					PDDL::Fluent* fl_ptr = task().fluents()[op_ptr->prec_vec()[k]];
					if ( !fl_ptr->enabled() ) continue;
					if ( !fluents.isset( op_ptr->prec_vec()[k] ) )
					{
						can_apply = false;
						break;
					}
				}				
				if (!can_apply) continue;
				task().print_operator(*it, os);
				os << "[" << *it<< "] ";
				fluents_to_add.add(task().useful_ops()[*it]->adds());
				num_displayed++;
				displayed.insert(*it);
				fp = false;
			}
		}
		std::cout << std::endl;
		std::cout << ++index << ":";
		fluents.add(fluents_to_add);
	}
	
	if(num_displayed != size()) 
	{
		std::cout << "Bug in relaxed plan. Could not support the following:" << std::endl;
		for(it = begin(); it != end(); it++) 
		{
			if( displayed.find(*it) == displayed.end()) 
			{
				task().print_operator(*it, os);
				os << ", missing pcs: ";
				for(unsigned i = 0; i < task().useful_ops()[*it]->prec_vec().size(); i++) 
				{
					if(!fluents.isset(task().useful_ops()[*it]->prec_vec()[i])) 
					{
						task().print_fluent(task().useful_ops()[*it]->prec_vec()[i], os);
						os << "which is " << ( task().fluents()[task().useful_ops()[*it]->prec_vec()[i]]->enabled() ? "enabled" : "disabled");
						os << ", ";
					}
				}
				os << std::endl;
			}
		}
	}
}

}
