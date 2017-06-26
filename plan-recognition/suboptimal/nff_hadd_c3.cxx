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
#include "nff_hadd_c3.hxx"

namespace NFF
{

H_Add::H_Add()
	: m_task( PDDL::Task::instance() ),
	m_action_cost(1),
	m_Nf( m_task.fluents().size() ),
	m_goal_op( m_task.end() )
{
	m_values.resize( m_Nf );
	m_h1_precs.resize( m_task.useful_ops().size() );
	m_best_supports.resize( m_Nf );
}

H_Add::~H_Add()
{
}

void H_Add::initialize_values( State* s )
{
	m_h1_precs[0] = 0;
	for ( unsigned i = 2; i < m_h1_precs.size(); i++ )
	{
		m_h1_precs[i] = std::numeric_limits<unsigned>::max();
	}

	if ( s == NULL )
	{
		PDDL::Operator* start_op = m_task.useful_ops()[m_task.start()];
		for ( unsigned i = 1; i < m_values.size(); i++ )
		{
			m_best_supports[i] = 0;
			m_values[i] = ( start_op->adds().isset(i) ? 0 : std::numeric_limits<unsigned>::max() );
		}
		
	}
	else
	{
		for ( unsigned i = 1; i < m_values.size(); i++ )
		{
			m_best_supports[i] = 0;
			m_values[i] = ( s->atom_set().isset(i) ? 0 : std::numeric_limits<unsigned>::max() ); 	
		}
	}
}

unsigned H_Add::eval_ff( Atom_Vec& s )
{
	unsigned h_ff = 0;
	if ( s.empty() ) return 0;
	unsigned len = eval(s);

	if ( len == 0 ) return 0;	
	if ( len == std::numeric_limits<unsigned>::max() ) return std::numeric_limits<unsigned>::max();

	std::vector< Layer* > graph;
	graph.resize( len+1 );

	for ( unsigned k = 0; k < graph.size(); k++ )
	{
		graph[k] = new Layer;
		graph[k]->Marked_True = new PDDL::Fluent_Set( m_task.fluents().size() );
	}

	for ( unsigned k = 0; k < s.size(); k++ )
		graph[value(s[k])]->Goals.push_back(s[k]);

	for ( unsigned k = len; k >= 1; k-- )
	{
		Atom_Vec& Gk = graph[k]->Goals;
		for ( unsigned i = 0; i < Gk.size(); i++ )
		{
			if ( graph[k]->Marked_True->isset( Gk[i] ) ) continue;
			unsigned best_supporter = m_task.useful_ops().size();
			unsigned min_diff = std::numeric_limits<unsigned>::max();
			Operator_Vec& supporters = m_task.added_by(Gk[i]);
			for ( unsigned j = 0; j < supporters.size(); j++ )
			{
				if ( value_op(supporters[j]) < min_diff )
				{
					min_diff = value_op(supporters[j]);
					best_supporter = supporters[j];
				}
			}
			assert ( best_supporter != m_task.useful_ops().size() );
			PDDL::Operator* best_sup_ptr = m_task.useful_ops()[best_supporter];
			Atom_Vec& sup_precs = best_sup_ptr->prec_vec();
			Atom_Vec& sup_adds = best_sup_ptr->add_vec();
			for ( unsigned j = 0; j < sup_precs.size(); j++ )
			{
				if ( value(sup_precs[j]) == 0 ) continue;
				if ( graph[k-1]->Marked_True->isset( sup_precs[j]) ) continue;
				graph[k-1]->Goals.push_back( sup_precs[j] );
			}
			for ( unsigned j = 0; j < sup_adds.size(); j++ )
			{
				graph[k]->Marked_True->set( sup_adds[j] );
				graph[k-1]->Marked_True->set( sup_adds[j] );
			}
			h_ff++;
		}
	}

	for ( unsigned k = 0; k < graph.size(); k++ )
	{
		delete graph[k]->Marked_True;
		delete graph[k];
	}

	return h_ff;
}

void H_Add::compute_with_persist( State* s, Atom_Vec* pw)
{
	initialize_values(s);
	
	Bool_Vec useable(m_task.useful_ops().size(), true);

	for ( unsigned o_idx = 2; o_idx < m_task.useful_ops().size(); o_idx++ )
	  {
	    if( m_task.reachable(o_idx) )
		{
		  PDDL::Operator* op_ptr = m_task.useful_ops()[o_idx];
		  for(unsigned p_idx = 0; p_idx < pw->size(); p_idx++)
		    {
			if( op_ptr->adds().isset(pw->at(p_idx)) || op_ptr->dels().isset(pw->at(p_idx)) || m_task.fast_op_edeletes(o_idx).isset( pw->at(p_idx) ) )
			  {
			    useable[o_idx]=false;
			    break;
			  }
		    }
		}
	  }
	
	bool fixed_point;

	do
	{
		fixed_point = true;
		for ( unsigned o_idx = 2; o_idx < m_task.useful_ops().size(); o_idx++ )
		{
		      if(!useable[o_idx])
			  continue;
		  
			PDDL::Operator* op = m_task.useful_ops()[o_idx];
			value_op(o_idx) = eval( op->prec_vec() );
			if ( value_op(o_idx) == std::numeric_limits<unsigned>::max() ) continue;
			for ( unsigned i = 0; i < op->add_vec().size(); i++ )
			{
				unsigned p = op->add_vec()[i];
				if ( value(p) == 0 ) continue;
				unsigned v = std::add(cost(), value_op(o_idx));
				if ( v < value(p) )
				{
					value(p) = v;
					support(p) = o_idx;
					fixed_point = false;
				}
			}
		}

	} while (!fixed_point);
	PDDL::Operator* op = m_task.useful_ops()[m_goal_op];
	value_op(m_goal_op) = eval( op->prec_vec() );
}

void H_Add::compute( State* s )
{
	initialize_values(s);
	bool fixed_point;
	do
	{
		fixed_point = true;

		for ( unsigned o_idx = 1; o_idx < m_task.useful_ops().size(); o_idx++ )
		{
			PDDL::Operator* op = m_task.useful_ops()[o_idx];
			value_op(o_idx) = eval( op->prec_vec() );
			if ( value_op(o_idx) == std::numeric_limits<unsigned>::max() ) continue;
			for ( unsigned i = 0; i < op->add_vec().size(); i++ )
			{
				unsigned p = op->add_vec()[i];
				if ( value(p) == 0 ) continue;
				unsigned v = std::add(cost(), value_op(o_idx));
				if ( v < value(p) )
				{
					value(p) = v;
					support(p) = o_idx;
					fixed_point = false;
				}
			}
		}

		/*
		for ( unsigned p = 1; p < m_task.fluents().size(); p++ )
		{
			if ( value(p) == 0 ) continue;
			Operator_Vec& Op = m_task.added_by(p);
			unsigned min_up = std::numeric_limits<unsigned>::max();
			unsigned best = 0;
			for ( unsigned i = 0; i < Op.size(); i++ )
			{
				PDDL::Operator* op = m_task.useful_ops()[Op[i]];
				unsigned v_op = eval( op->prec_vec() );
				unsigned h_up = std::add( cost(), v_op );
				if ( h_up < min_up )
				{
					min_up = h_up;
					best = Op[i];
				}
			}
			if ( min_up < value(p) )
			{
				value(p) = min_up;
				support(p) = best;
				fixed_point = false;
			}
		}
		*/
	} while (!fixed_point);
	/*
	std::ofstream out_prec_vals( "h1.prec_vals" );
	for ( unsigned op = 1; op < m_task.useful_ops().size(); op++ )
	{
		out_prec_vals << "h¹( ";
		m_task.print_operator( m_task.useful_ops()[op], out_prec_vals );
		out_prec_vals << " )= " << value_op(op) << std::endl;
	}
	out_prec_vals.close();
	*/
}


}
