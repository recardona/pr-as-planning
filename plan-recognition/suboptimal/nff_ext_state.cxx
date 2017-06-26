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
#include "nff_ext_state.hxx"

namespace NFF
{

PDDL::Task&	Ext_State::sm_task = PDDL::Task::instance();

Ext_State::Ext_State()
{
	m_model.resize( task().fluents().size() );
	for ( unsigned i = 0; i < m_model.size(); i++ )
		m_model[i] = l_Undef;
}

Ext_State::Ext_State( std::vector<unsigned>& atoms, bool assume_false )
{
	m_model.resize( task().fluents().size() );
	for ( unsigned i = 0; i < m_model.size(); i++ )
		m_model[i] = ( assume_false ? l_False : l_Undef );
	
	for ( unsigned i = 0; i < atoms.size(); i++ )
		m_model[atoms[i]] = l_True;
}

Ext_State::Ext_State( State& s )
{
	m_model.resize( task().fluents().size() );
	for ( unsigned i = 0; i < m_model.size(); i++ )
		m_model[i] = l_False;
	
	for ( unsigned i = 0; i < s.atom_vec().size(); i++ )
		m_model[s.atom_vec()[i]] = l_True;
}

Ext_State::Ext_State( Ext_State& s )
{
	m_model.assign( s.m_model.begin(), s.m_model.end() );
}

Ext_State::~Ext_State()
{
}

void Ext_State::update( unsigned op )
{
	PDDL::Operator* p_op = task().useful_ops()[op];

	for ( unsigned i = 0; i < p_op->add_vec().size(); i++ )
		m_model[ p_op->add_vec()[i] ] = l_True;
	for ( unsigned i = 0; i < p_op->del_vec().size(); i++ )
		m_model[ p_op->del_vec()[i] ] = l_False;
}


void Ext_State::update_wo_neg( unsigned op )
{
	PDDL::Operator* p_op = task().useful_ops()[op];

	for ( unsigned i = 0; i < p_op->add_vec().size(); i++ )
		m_model[ p_op->add_vec()[i] ] = l_True;
	for ( unsigned i = 0; i < p_op->del_vec().size(); i++ )
		m_model[ p_op->del_vec()[i] ] = l_Undef;
}

void Ext_State::assign( Atom_Vec& a, lbool v )
{
	for ( unsigned i = 0; i < a.size(); i++ )
		m_model[ a[i] ] = v;
}

void Ext_State::unassign( Atom_Vec& a )
{
	for ( unsigned i = 0; i < a.size(); i++ )
		m_model[ a[i] ] = l_Undef;
}

bool Ext_State::included( Atom_Vec& a )
{
	for ( unsigned i = 0; i < a.size(); i++ )
		if ( m_model[a[i]] != l_True ) return false;
	return true;
}

bool Ext_State::union_of( Ext_State& s1, Ext_State& s2 )
{
	for ( unsigned p = 1; p < task().fluents().size(); p++ )
	{
		lbool l1 = s1[p], l2 = s2[p];
		if ( l1 == l2 )
		{
			m_model[p] = l1;
		}	
		else
		{
			if ( l1 != l_Undef && l2 != l_Undef )
				return false; // inconsistent truth values
			else
			{
				if ( l2 == l_Undef ) 
					m_model[p] = l1;
				else 
					m_model[p] = l2;
			}
		}
	}
	return true;
}

bool Ext_State::union_with( Ext_State& s )
{
	for ( unsigned p = 1; p < task().fluents().size(); p++ )
	{
		if ( m_model[p] == l_Undef ) 
			m_model[p] = s[p];
		else
		{
			if ( s[p] == l_Undef || m_model[p] == s[p]) continue;
			return false; // inconsistent truth values
		}
	}
	return true;
}

void Ext_State::intersection_of( Ext_State& s1, Ext_State& s2 )
{
	for ( unsigned p = 1; p < task().fluents().size(); p++ )
	{
		lbool l1 = s1[p], l2 = s2[p];
		if ( l1 == l2 ) m_model[p] = l1;
	}
}

void Ext_State::intersection_with( Ext_State& s1)
{
	for ( unsigned p = 1; p < task().fluents().size(); p++ )
	{
		if ( m_model[p] != s1[p] ) m_model[p] = l_Undef;
	}
}

bool Ext_State::empty()
{
	for ( unsigned p = 0; p < m_model.size(); p++ )
	{
		if ( m_model[p] != l_Undef ) return false;
	}
	return true;
}

bool Ext_State::operator==( Ext_State& s )
{
	if ( s.m_model.size() != m_model.size() ) return false;

	for ( unsigned i = 0; i < m_model.size(); i++ )
		if ( m_model[i] != s[i] ) return false;

	return true;
}

void Ext_State::intersection_with( PDDL::Fluent_Set& lits )
{
	for ( unsigned i = 1; i < m_model.size(); i++ )
	{
		if ( m_model[i] == l_Undef ) continue;
		Lit li = ( m_model[i] == l_True ? Lit(i) : Lit(i, true) );
		if (!lits.isset(toInt(li))) m_model[i] = l_Undef;
	}
	/*
	// atoms must be sorted
	for ( unsigned j = 1; j < var(lits[0]); j++ )
		m_model[j] = l_Undef;

	unsigned idx = var(lits[0]);
	if ( m_model[idx] == l_True && !sign(lits[0]) )
		m_model[idx] = l_True;
	else if ( m_model[idx] == l_False && sign(lits[0] ) )
		m_model[idx] = l_False;
	else
		m_model[idx] = l_Undef;


	for ( unsigned i = 1; i < lits.size(); i++ )
	{
		for ( unsigned j = var(lits[i-1]) + 1; j < var(lits[i]); j++ )
		{
			m_model[j] = l_Undef;
		}
		unsigned idx = var(lits[i]);
		if ( m_model[idx] == l_True && !sign(lits[i]) )
			m_model[idx] = l_True;
		else if ( m_model[idx] == l_False && sign(lits[i] ) )
			m_model[idx] = l_False;
		else
			m_model[idx] = l_Undef;
	}
	for ( unsigned j = var(lits.back()) + 1 ; j < m_model.size(); j++ )
		m_model[j] = l_Undef;
	*/
}

void Ext_State::print()
{
	std::cout << "{";
	for ( unsigned i = 1; i < m_model.size(); i++ )
	{
		if ( m_model[i] == l_Undef ) continue;
		if ( m_model[i] == l_True )
		{
			task().print_fluent( task().fluents()[i], std::cout );
			std::cout << ", ";
		}
		if ( m_model[i] == l_False )
		{
			std::cout << "~";
			task().print_fluent( task().fluents()[i], std::cout );
			std::cout << ", ";
		}

	}
	std::cout << "}" << std::endl;
}

}
