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
#include "nff_h_add.hxx"

namespace NFF
{

Additive_Heuristic::Additive_Heuristic()
	: m_task( PDDL::Task::instance() ),
	m_Nf( m_task.fluents().size() ),
	m_goal_op( m_task.end() ),
	m_compute_classic( false )
{
	m_values.resize( m_Nf );
	m_h1_precs.resize( m_task.useful_ops().size() );
	m_best_supports.resize( m_Nf );
	m_metric_values.resize( m_Nf );
	m_metric_h1_precs.resize( m_task.useful_ops().size() );
	m_metric_best_supports.resize( m_Nf );

}

Additive_Heuristic::~Additive_Heuristic()
{
}

void Additive_Heuristic::initialize_values( State* s )
{
	m_h1_precs[0] = 0;
	m_metric_h1_precs[0] = 0.0f;
	for ( unsigned i = 2; i < m_h1_precs.size(); i++ )
	{
		m_h1_precs[i] = std::numeric_limits<unsigned>::max();
		m_metric_h1_precs[i] = std::numeric_limits<float>::max();
	}

	if ( s == NULL )
	{
		PDDL::Operator* start_op = m_task.useful_ops()[m_task.start()];
		for ( unsigned i = 1; i < m_values.size(); i++ )
		{
			m_best_supports[i] = m_metric_best_supports[i] = 0;
			m_values[i] = ( start_op->adds().isset(i) ? 0 : std::numeric_limits<unsigned>::max() );
			m_metric_values[i] = ( start_op->adds().isset(i) ? 0.0f : std::numeric_limits<float>::max() );
		}
		
	}
	else
	{
		for ( unsigned i = 1; i < m_values.size(); i++ )
		{
			m_best_supports[i] = m_metric_best_supports[i] = 0;
			m_values[i] = ( s->atom_set().isset(i) ? 0 : std::numeric_limits<unsigned>::max() ); 	
			m_metric_values[i] = ( s->atom_set().isset(i) ? 0.0f : std::numeric_limits<float>::max() ); 	
		}
	}
}

void Additive_Heuristic::compute( State* s )
{
	initialize_values(s);
	
	bool fixed_point;
	PDDL::Operator* op_end_ptr = m_task.useful_ops()[m_goal_op];
	//if ( m_compute_classic )
	//{
		do
		{
			fixed_point = true;
			for ( unsigned o_idx = 2; o_idx < m_task.useful_ops().size(); o_idx++ )
			{
				PDDL::Operator* op = m_task.useful_ops()[o_idx];
				value_op(o_idx) = eval( op->prec_vec() );
				if ( value_op(o_idx) == std::numeric_limits<unsigned>::max() ) continue;
				for ( unsigned i = 0; i < op->add_vec().size(); i++ )
				{
					unsigned p = op->add_vec()[i];
					if ( value(p) == 0) continue;
					unsigned v = std::add(1u, value_op(o_idx));
					if ( v < value(p) )
					{
						value(p) = v;
						support(p) = o_idx;
						fixed_point = false;
					}
				}
			}
	
		} while (!fixed_point);
		value_op(m_goal_op) = eval( op_end_ptr->prec_vec() );
		//return;
	//}

	do
	{
		fixed_point = true;
		for ( unsigned o_idx = 2; o_idx < m_task.useful_ops().size(); o_idx++ )
		{
			PDDL::Operator* op = m_task.useful_ops()[o_idx];
			metric_value_op(o_idx) = metric_eval( op->prec_vec() );
			if ( metric_value_op(o_idx) == std::numeric_limits<float>::max() ) continue;
			for ( unsigned i = 0; i < op->add_vec().size(); i++ )
			{
				unsigned p = op->add_vec()[i];
				if ( metric_value(p) <= 1e-7 ) continue;
				float v = std::add( m_task.op_cost(o_idx), metric_value_op(o_idx));
				if ( v < metric_value(p) )
				{
					metric_value(p) = v;
					metric_support(p) = o_idx;
					fixed_point = false;
				}
			}
		}
	} while (!fixed_point);
	metric_value_op(m_goal_op) = metric_eval( op_end_ptr->prec_vec() );

	
	#ifndef NDEBUG
	std::ofstream out( "hadd.atoms" );
	for ( unsigned p = 1; p < m_task.fluents().size(); p++ )
	{
		out << "hadd( ";
		m_task.print_fluent( p, out );
		out << ") = ";
		out << metric_value(p);
		out << std::endl;
	}	
	out << "GOAL = " << metric_value_op(m_goal_op) << std::endl;
	out.close();
	#endif
}


}
