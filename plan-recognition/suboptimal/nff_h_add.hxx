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
#ifndef __NFF_HADD__
#define __NFF_HADD__

#include "ext_math.hxx"
#include "PDDL.hxx"
#include "nff.hxx"
#include "nff_state.hxx"

namespace NFF
{

class Additive_Heuristic
{
public:
	Additive_Heuristic();
	~Additive_Heuristic();

	float& metric_value( unsigned p )
	{
		return m_metric_values[p];
	}
	
	unsigned& value( unsigned p )
	{
		return m_values[p];
	}

	unsigned  eval( Atom_Vec& s )
	{
		unsigned v = 0;
		for ( unsigned i = 0; i < s.size(); i++ )
		{
			v = std::add( v, value(s[i]) );
		}	
		return v;
	}

	float metric_eval( Atom_Vec& s )
	{
		float v = 0.0f;
		for ( unsigned i = 0; i < s.size(); i++ )
			v = std::add( v, metric_value(s[i]) );
		return v;
	}

	unsigned& support( unsigned op )
	{
		return m_best_supports[op];
	}	

	unsigned& metric_support( unsigned op )
	{
		return m_metric_best_supports[op];
	}

	unsigned& value_op( unsigned op )
	{
		return m_h1_precs[op];
	}

	float& metric_value_op( unsigned op )
	{
		return m_metric_h1_precs[op];
	}

	void compute( State* s = NULL );

	template <typename Node_Type>
	void compute( Node_Type* n )
	{
		compute( n->s );
	}

	void initialize_values( State* s );

	void set_goal( unsigned op ) { m_goal_op = op; }
	void compute_classic_hadd() { m_compute_classic = true; }
protected:


protected:
	PDDL::Task&				m_task;
	std::vector<unsigned>			m_values;
	std::vector<unsigned>			m_h1_precs;
	std::vector<unsigned>			m_best_supports;
	std::vector<float>			m_metric_values;
	std::vector<float>			m_metric_h1_precs;
	std::vector<unsigned>			m_metric_best_supports;			
	unsigned				m_action_cost;
	unsigned				m_Nf;
	unsigned				m_goal_op;
	bool					m_compute_classic;
};

}

#endif // nff_h_add.hxx
