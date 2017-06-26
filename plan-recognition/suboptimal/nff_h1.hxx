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
#ifndef __NFF_H1__
#define __NFF_H1__

#include "ext_math.hxx"
#include "PDDL.hxx"
#include "nff.hxx"
#include "nff_state.hxx"

namespace NFF
{

class Supports_Table;

class H1
{
public:
	struct Layer
	{
		Atom_Vec 		Goals;
		PDDL::Fluent_Set*	Marked_True;
	};



	H1();
	~H1();
	
	unsigned& value( unsigned p )
	{
		return m_values[p];
	}

	unsigned  cost()
	{
		return m_action_cost;
	}
	
	unsigned  eval( Atom_Vec& s )
	{
		unsigned v = 0;
		for ( unsigned i = 0; i < s.size(); i++ )
		{
			v = std::max( v, value(s[i]) );
		}	
		return v;
	}

	unsigned eval_ff( Atom_Vec& s );

	unsigned& support( unsigned op )
	{
		return m_best_supports[op];
	}	

	unsigned& value_op( unsigned op )
	{
		return m_h1_precs[op];
	}

	unsigned operator()( State* s )
	{
		return eval( s->atom_vec() );
	}

	void compute( State* s = NULL );
  
      void compute_with_persist( State* s, Atom_Vec* pw);

	template <typename Node_Type>
	void compute( Node_Type* n )
	{
		compute( n->s );
	}

  	template <typename Node_Type>
	void compute_with_persist( Node_Type* n, Atom_Vec* pw )
	{
	  compute_with_persist( n->s, pw );
	}
	void initialize_values( State* s );
	
	void compute_difficulties( );

protected:


protected:
	PDDL::Task&				m_task;
	std::vector<unsigned>			m_values;
	std::vector<unsigned>			m_h1_precs;
	std::vector<unsigned>			m_best_supports;
	unsigned				m_action_cost;
	unsigned				m_Nf;
	std::vector<unsigned>			m_difficulties;

};

}

#endif // nff_h1.hxx
