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
#ifndef __NFF_H2__
#define __NFF_H2__

#include "ext_math.hxx"
#include "PDDL.hxx"
#include "pddl_fluent_set.hxx"
#include "nff.hxx"
#include "nff_state.hxx"

namespace NFF
{

class H1;

// The h² heuristic for NFF
class H2
{
public:
	H2();
	~H2();
	unsigned& value( unsigned p, unsigned q )
	{
		return m_values[p*m_Nf + q];
	}

	unsigned& support( unsigned p, unsigned q )
	{
		return m_supports[p*m_Nf + q];
	}

	unsigned value( unsigned p, unsigned q ) const
	{
		return m_values[p*m_Nf + q];
	}

	unsigned& value( unsigned p )
	{
		return m_values[p*m_Nf + p];
	}

	unsigned value( unsigned p ) const
	{
		return m_values[p*m_Nf + p];
	}
	
	// TODO: Will need to change this whenever we go for supporting action
	// costs
	unsigned cost() 
	{
		return m_action_cost;
	}

	// compute from the initial state	
	void compute(State* s = NULL);
	void compute_only_mutexes(State* s = NULL);

	unsigned eval( Atom_Vec& s )
	{
		unsigned v = 0;
		for ( unsigned i = 0; i < s.size(); i++ )
			for ( unsigned j = i; j < s.size(); j++ )
				v = std::max( v, value( s[i], s[j] ) );

		return v;
	}

	unsigned& value_op( unsigned op )
	{
		return m_h2_precs[op];
	}

	unsigned eval( Atom_Pair_Vec& s, std::vector<unsigned>& h2_vals );

	unsigned operator()( State* s )
	{
		return eval( s->atom_vec() );
	}

	bool extract_op_reachability_info();

	void initialize_values(State* s);	

	void compute_e_deletes();
protected:
	std::vector<unsigned>					m_values;
	std::vector< unsigned >					m_supports;
	unsigned						m_action_cost;
	PDDL::Task&						m_task;
	unsigned						m_Nf;
	std::vector<unsigned>					m_h2_precs;
};

}

#endif // nff_h2.hxx
