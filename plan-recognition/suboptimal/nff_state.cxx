/*
    Miguel Ramirez, Nir Lipovetzky, Hector Geffner
    C^3: A planner for the sequential, satisficing track of the IPC-6
    Copyright (C) 2008  

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "nff_state.hxx"
#include <iostream>

namespace NFF
{

PDDL::Task& State::sm_task = PDDL::Task::instance();

State::State()
	: m_atomset( NULL ), m_atoms( NULL )
{
}

State::State( std::vector<unsigned>& atoms )
{
	m_atoms = new std::vector<unsigned>( atoms.begin(), atoms.end() );
	m_atomset = new PDDL::Fluent_Set( task().fluents().size()+1 );	
	for ( unsigned i = 0; i < atom_vec().size(); i++ )
		atom_set().set( atom_vec()[i] );
}

State::State( State& s )
{
	m_atoms = new std::vector<unsigned>( s.atom_vec().begin(), s.atom_vec().end() );
	m_atomset = new PDDL::Fluent_Set( task().fluents().size()+1 );	
	for ( unsigned i = 0; i < atom_vec().size(); i++ )
		atom_set().set( atom_vec()[i] );
	m_enabled_atoms.assign( s.enabled_atoms().begin(), s.enabled_atoms().end() );
}

State::~State()
{
	delete m_atoms;
	delete m_atomset;
}

State* State::make_initial_state()
{
	return new State( State::sm_task.useful_ops()[sm_task.start()]->add_vec() );
}

State* State::make_goal_state()
{
	return new State(  State::sm_task.useful_ops()[sm_task.end()]->prec_vec() );
}

bool State::is_goal()
{
	PDDL::Operator* op = task().useful_ops()[task().end()];

	//for ( unsigned k = 0; k < enabled_atoms().size(); k++ )
	//	task().fluents()[ enabled_atoms()[k] ]->enable();

	for ( unsigned i = 0; i < op->prec_vec().size(); i++ )
		if ( task().fluents()[op->prec_vec()[i]]->enabled() && !atom_set().isset( op->prec_vec()[i] ) ) return false;

	
	//for ( unsigned k = 0; k < enabled_atoms().size(); k++ )
	//	task().fluents()[ enabled_atoms()[k] ]->disable();
	return true;
}

bool State::can_apply( unsigned op_idx )
{
	if ( !task().reachable( op_idx ) ) return false;

	PDDL::Operator* op = task().useful_ops()[op_idx];


	for ( unsigned i = 0; i < op->prec_vec().size(); i++ )
		if ( !atom_set().isset( op->prec_vec()[i] ) ) return false;


	if ( task().is_obs(op_idx) )
	{
		unsigned explained_atom_added = 0;
		for ( unsigned k = 0; 
			k < op->add_vec().size(); 
			k++ )
			if (  task().is_explained(op->add_vec()[k]) )
			{
				explained_atom_added = 
					op->add_vec()[k];
				break;
			}
			if (  atom_set().isset(explained_atom_added) ) 
				return false;
	}	


	return true;	
}

State* State::apply( unsigned op_idx )
{
	PDDL::Operator* op = task().useful_ops()[op_idx];

	std::vector<unsigned>  new_atoms;
	for ( unsigned i = 0; i < atom_vec().size(); i++ )
	{
		if ( op->dels().isset( atom_vec()[i] ) ) continue;
		new_atoms.push_back( atom_vec()[i] );
	}	
	for ( unsigned i = 0; i < op->add_vec().size(); i++ )
	{
		if ( atom_set().isset( op->add_vec()[i] ) ) continue;
		new_atoms.push_back( op->add_vec()[i] );
	}

	State* succ = new State( new_atoms );

	succ->enabled_atoms().assign( enabled_atoms().begin(), enabled_atoms().end() );

	return succ;
}

void State::print( std::ostream& os )
{
	os << "{";
	for ( unsigned p = 1; p < task().fluents().size(); p++ )
	{
		if ( atom_set().isset(p) )
		{
			task().print_fluent( task().fluents()[p], os );
			os << " ";
		}
	
	}
	/*
	for ( unsigned i = 0; i < atom_vec().size(); i++ )
	{
		task().print_fluent( task().fluents()[atom_vec()[i]], os );
		if ( i < atom_vec().size()-1 )
			os << ", ";
	}
	*/
	os << "}";
}

void State::print_indices( std::ostream& os )
{
	os << "{";
	for ( unsigned p = 1; p < task().fluents().size(); p++ )
	{
		if ( atom_set().isset(p) )
		{
			task().print_fluent( task().fluents()[p], os );
			os << " ";
		}
	
	}

	/*
	for ( unsigned i = 0; i < atom_vec().size(); i++ )
	{
		os << atom_vec()[i];
		if ( i < atom_vec().size()-1 )
			os << ", ";
	}
	*/
	os << "}";
}



}
