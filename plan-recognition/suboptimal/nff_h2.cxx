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
#include "nff_h2.hxx"
#include "nff_h1.hxx"
#include "square_matrix.hxx"
#include "nff_options.hxx"

namespace NFF
{

unsigned	Atom_Pair::_nF = 0;

H2::H2()
	: m_action_cost(1), m_task( PDDL::Task::instance() ), m_Nf( m_task.fluents().size() )
{
	m_values.resize( m_Nf * m_Nf );
	m_supports.resize( m_Nf * m_Nf );
	Atom_Pair::_nF = m_Nf;
	m_h2_precs.resize( m_task.useful_ops().size() );
}

H2::~H2()
{
}

void H2::initialize_values( State* s )
{
	m_h2_precs[0] = 0;
	for ( unsigned i = 2; i < m_h2_precs.size(); i++ )
		m_h2_precs[i] = std::numeric_limits<unsigned>::max();
	for ( unsigned i = 0; i < m_supports.size(); i++ )
		m_supports[i] = 0;
	if ( s == NULL )
	{
		PDDL::Operator* start_op = m_task.useful_ops()[m_task.start()];
	
		for ( unsigned k = 0; k < m_values.size(); k++ )
			m_values[k] = std::numeric_limits<unsigned>::max();
	
		for ( unsigned i = 0; i < start_op->add_vec().size(); i++ )
		{
			unsigned p = start_op->add_vec()[i];
			value(p,p) = 0;
			for ( unsigned j = i+1; j < start_op->add_vec().size(); j++ )
			{
				unsigned q = start_op->add_vec()[j];
				value(p,q) = value(q,p) = 0;
			}
		}
	}
	else
	{
		for ( unsigned k = 0; k < m_values.size(); k++ )
			m_values[k] = std::numeric_limits<unsigned>::max();
		for ( unsigned i = 0; i < s->atom_vec().size(); i++ )
		{
			unsigned p = s->atom_vec()[i];
			value(p,p) = 0;
			for ( unsigned j = i+1; j < s->atom_vec().size(); j++ )
			{
				unsigned q = s->atom_vec()[j];
				value(p,q) = value(q,p) = 0;
			}
		}
	}
}

void H2::compute_only_mutexes(State* s )
{
	initialize_values(s);

	bool fixed_point;
	do
	{
		fixed_point = true;
		for( unsigned o_idx = 2; o_idx < m_task.useful_ops().size(); o_idx++ )
		{
			PDDL::Operator* o = m_task.useful_ops()[o_idx];
			value_op(o_idx) = eval( o->prec_vec() );
			if ( value_op(o_idx) == std::numeric_limits<unsigned>::max() ) continue;
			for ( unsigned i = 0; i < o->add_vec().size(); i++ )
			{
				unsigned p = o->add_vec()[i];
				for ( unsigned j = i; j < o->add_vec().size(); j++ )
				{
					unsigned q = o->add_vec()[j];
					if ( value(p,q) == 0 ) continue;
					unsigned v = value_op(o_idx);
					if ( v < value(p,q) )
					{ 	
						value(p,q) = value(q,p) = v;
						support(p,q) = support(q,p) = o_idx;
						fixed_point = false;
					}
				}
				for ( unsigned r = 1; r < m_Nf; r++ )
				{
					if ( o->adds().isset(r) || o->dels().isset(r) || value( p, r ) == 0) continue;
					// h²( Pre(o) \cup {r})
					unsigned h2_pre_noop = std::max( value_op(o_idx), value(r,r) );
					// Atom r still unreachable, cutting up the computation
					if ( h2_pre_noop == std::numeric_limits<unsigned>::max() ) continue;
					for ( unsigned j = 0; j < o->prec_vec().size(); j++ )
					{
						unsigned s = o->prec_vec()[j];
						h2_pre_noop = std::max( h2_pre_noop, value(r,s) );
					}
					unsigned v = h2_pre_noop;
					if ( v < value(p,r) )
					{
						value(p,r) = value(r,p) = v;
						support(p,r) = support(r,p) = o_idx;
						fixed_point = false;
					}
				}
			}
		}
	} while ( !fixed_point );

}

void H2::compute(State* s)
{
	initialize_values(s);

	bool fixed_point;
	do
	{
		fixed_point = true;
		for( unsigned o_idx = 2; o_idx < m_task.useful_ops().size(); o_idx++ )
		{
			PDDL::Operator* o = m_task.useful_ops()[o_idx];
			value_op(o_idx) = eval( o->prec_vec() );
			if ( value_op(o_idx) == std::numeric_limits<unsigned>::max() ) continue;
			for ( unsigned i = 0; i < o->add_vec().size(); i++ )
			{
				unsigned p = o->add_vec()[i];
				for ( unsigned j = i; j < o->add_vec().size(); j++ )
				{
					unsigned q = o->add_vec()[j];
					if ( value(p,q) == 0 ) continue;
					unsigned v = std::add( cost(), value_op(o_idx) );
					if ( v < value(p,q) )
					{ 	
						value(p,q) = value(q,p) = v;
						support(p,q) = support(q,p) = o_idx;
						fixed_point = false;
					}
				}
				for ( unsigned r = 1; r < m_Nf; r++ )
				{
					if ( o->adds().isset(r) || o->dels().isset(r) || value( p, r ) == 0) continue;
					// h²( Pre(o) \cup {r})
					unsigned h2_pre_noop = std::max( value_op(o_idx), value(r,r) );
					// Atom r still unreachable, cutting up the computation
					if ( h2_pre_noop == std::numeric_limits<unsigned>::max() ) continue;
					for ( unsigned j = 0; j < o->prec_vec().size(); j++ )
					{
						unsigned s = o->prec_vec()[j];
						h2_pre_noop = std::max( h2_pre_noop, value(r,s) );
					}
					unsigned v = std::add( cost(), h2_pre_noop );
					if ( v < value(p,r) )
					{
						value(p,r) = value(r,p) = v;
						support(p,r) = support(r,p) = o_idx;
						fixed_point = false;
					}
				}
			}
		}
	} while ( !fixed_point );
}

bool H2::extract_op_reachability_info()
{
	m_h2_precs[m_task.end()] = eval( m_task.useful_ops()[m_task.end()]->prec_vec() );
	m_task.set_start_time_lb( m_h2_precs );

	return m_task.reachable( m_task.end() );
}


unsigned H2::eval( Atom_Pair_Vec& s, std::vector<unsigned>& h2_vals )
{
	unsigned v = 0;
	for ( unsigned i = 0; i < s.size(); i++ )
	{
		h2_vals[i] = value( s[i].p(), s[i].q() );
		v = std::max( v, h2_vals[i] );
	}
	return v;
}


void H2::compute_e_deletes()
{
	std::vector<PDDL::Operator*>& ops = m_task.useful_ops();

	for ( unsigned p = 1 ; p < m_Nf; p++ )
	{
		for ( unsigned o = 0; o < ops.size(); o++ )
		{
			bool is_edelete = false;
			PDDL::Operator* op = ops[o];
			std::vector<unsigned>& op_adds = op->add_vec();
			for ( unsigned i = 0; i < op_adds.size(); i++ )
			{
				unsigned q = op_adds[i];
				if ( value(p,q) == std::numeric_limits<unsigned>::max() )
				{
					is_edelete = true;
					m_task.e_deleted_by( p ).push_back( o );
					m_task.op_edeletes( o ).push_back( p );
					m_task.fast_op_edeletes(o).set(p);
					break;
				}
			}
			if ( is_edelete ) continue;
		
			std::vector<unsigned>& op_prec = op->prec_vec();
			for ( unsigned i = 0; i < op_prec.size(); i++ )
			{
				unsigned r = op_prec[i];
				if ( !op->adds().isset(p) && value( p, r ) == std::numeric_limits<unsigned>::max() )
				{
					m_task.e_deleted_by( p ).push_back( o );
					m_task.op_edeletes( o ).push_back( p );
					m_task.fast_op_edeletes(o).set(p);
					break;
				}
			}
		}
	}
}


}
