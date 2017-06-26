#include "nff_const_h1.hxx"

namespace NFF
{

Constrained_H1::Constrained_H1()
	: m_task( PDDL::Task::instance() ),
	m_action_cost(1),
	m_Nf( m_task.fluents().size() ),
	m_end_op( m_task.end() )
{
	m_values.resize( m_Nf );
	m_op_values.resize( m_task.useful_ops().size() );
	m_constraints.resize( m_task.useful_ops().size() );
	m_difficulties.resize( m_task.useful_ops().size() );
}

Constrained_H1::~Constrained_H1()
{
}

void Constrained_H1::collect_constraints( State* s, PDDL::Fluent_Set& protected_atoms, Support_List& pending )
{
  
  for ( unsigned o = 0; o < m_task.useful_ops().size(); o++ )
    m_constraints[o].clear();

	for ( Support_List::iterator l = pending.begin();
		l != pending.end(); l++ )
	{
		Operator_Vec& adding = m_task.added_by(l->p);
		Operator_Vec& deleting = m_task.deleted_by(l->p);
		Operator_Vec& e_deleting = m_task.e_deleted_by(l->p);
	
		for ( unsigned k = 0; k < adding.size(); k++ )
			if ( m_task.reachable( adding[k] ) )
			  {
			    if ( !m_task.equal_effects( l->a, adding[k] ) )
				{
				m_constraints[adding[k]].insert( l->a );

				}
			  }
		for ( unsigned k = 0; k < e_deleting.size(); k++ )
			if ( m_task.reachable( e_deleting[k] ) )
			{
				if ( !m_task.equal_effects( l->a, e_deleting[k] ) )
				  {
					m_constraints[e_deleting[k]].insert( l->a );

				  }
			}
		for ( unsigned k = 0; k < deleting.size(); k++ )
			if ( m_task.reachable( deleting[k] ) )
			{
				if ( !m_task.equal_effects( l->a, deleting[k] ) )
				{
					m_constraints[deleting[k]].insert( l->a );

				}
			}
	}
}

bool Constrained_H1::can_apply( PDDL::Fluent_Set& P, unsigned op_idx )
{
	PDDL::Operator* op = m_task.useful_ops()[ op_idx ];
	Atom_Vec& op_prec = op->prec_vec();
	for ( unsigned k = 0; k < op_prec.size(); k++ )
	{
		if ( !P.isset( op_prec[k] ) )
			return false;
	}
	return true;
}

bool Constrained_H1::consistent( unsigned i, unsigned op_idx )
{
	for ( Operator_Set::iterator pc = m_constraints[op_idx].begin();
		pc != m_constraints[op_idx].end(); pc++ )
	{
		if ( i <= value_op( *pc ) ) return false;
	}
	return true;
}

void Constrained_H1::compute_difficulties()
{
	for ( unsigned o = 0; o < m_task.useful_ops().size(); o++ )
	{
		PDDL::Operator* op = m_task.useful_ops()[o];
		Atom_Vec& precs = op->prec_vec();
		if ( value_op( o ) == std::numeric_limits<unsigned>::max() )
		{
			m_difficulties[o] = std::numeric_limits<unsigned>::max();
			continue;
		}
		m_difficulties[o] = 0;
		for ( unsigned i = 0; i < precs.size(); i++ )
			m_difficulties[o] = std::add( value( precs[i]), m_difficulties[o] );
	}
}

unsigned Constrained_H1::eval_ff( Atom_Vec& s )
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

	compute_difficulties();

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
				if ( value_op(supporters[j]) > k-1 ) continue;
				if ( std::add( (unsigned)m_task.op_cost(supporters[j]), m_difficulties[supporters[j]] ) < min_diff )
				{
					min_diff = std::add( (unsigned)m_task.op_cost(supporters[j]), m_difficulties[supporters[j]]);
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
			h_ff = std::add( (unsigned)m_task.op_cost(best_supporter), h_ff );
		}
	}

	for ( unsigned k = 0; k < graph.size(); k++ )
	{
		delete graph[k]->Marked_True;
		delete graph[k];
	}

	return h_ff;
}

void Constrained_H1::compute( State* s, PDDL::Fluent_Set& protected_atoms, Support_List& pending  )
{
	collect_constraints(s, protected_atoms, pending );
	for ( unsigned p = 1; p < m_task.fluents().size(); p++ )
		value(p) = std::numeric_limits<unsigned>::max();
	for ( unsigned o = 0; o < m_task.useful_ops().size(); o++ )
		value_op(o) = std::numeric_limits<unsigned>::max();

	PDDL::Fluent_Set P( m_task.fluents().size() + 1 );
	Operator_Vec A;
	Operator_Vec A2;
	
	unsigned i = 0;
	for ( unsigned k = 0; k < s->atom_vec().size(); k++ )
	{
		unsigned p = s->atom_vec()[k];
		P.set( p );	
		value( p ) = std::min( i, value(p) );
	}

	if ( can_apply( P, m_end_op ) )
	{
		value_op( m_task.end() ) = i;
		return;
	}
	for ( unsigned op_idx = 2; op_idx < m_task.useful_ops().size(); op_idx++ )
	{
		if ( !can_apply(P, op_idx) ) continue;
		if ( !consistent( i, op_idx ) ) continue;
		A.push_back( op_idx );
		value_op( op_idx ) = i;
	}

	while ( true )
	{
		i++;
		bool new_atoms = false;
		for ( unsigned k = 0; k < A.size(); k++ )
		{
			PDDL::Operator* a = m_task.useful_ops()[A[k]];
			Atom_Vec& adds = a->add_vec();
			for ( unsigned j = 0; j < adds.size(); j++ )
			{
				if ( P.isset( adds[j] ) ) continue;
				P.set( adds[j] );
				value( adds[j] ) = std::min( value(adds[j]), i );
				new_atoms = true;
			}
		}

		
		if ( can_apply( P, m_end_op ) )
		{
			value_op( m_task.end() ) = i;
			return;
		}
		else
		{
			if (!new_atoms ) return;
		}
		A.clear();
		for ( unsigned op_idx = 2; op_idx < m_task.useful_ops().size(); op_idx++ )
		{
			if ( !can_apply(P, op_idx) ) continue;
			if ( !consistent( i, op_idx ) ) continue;
			A.push_back( op_idx );
			value_op( op_idx ) = std::min( i, value_op(op_idx) );
		}
		
	}
}

void Constrained_H1::compute_with_persist( State* s, PDDL::Fluent_Set& protected_atoms, Support_List& pending, Atom_Vec* pw  )
{
	collect_constraints(s, protected_atoms, pending );
	for ( unsigned p = 1; p < m_task.fluents().size(); p++ )
		value(p) = std::numeric_limits<unsigned>::max();
	for ( unsigned o = 0; o < m_task.useful_ops().size(); o++ )
		value_op(o) = std::numeric_limits<unsigned>::max();

	PDDL::Fluent_Set P( m_task.fluents().size() + 1 );
	Operator_Vec A;
	Operator_Vec A2;
	
	
	Bool_Vec useable(m_task.useful_ops().size(), true);


	for ( unsigned o_idx = 2; o_idx < m_task.useful_ops().size(); o_idx++ )
	  {
	    if( m_task.reachable(o_idx) )
		{
		  PDDL::Operator* op_ptr = m_task.useful_ops()[o_idx];
		  for(unsigned p_idx = 0; p_idx < pw->size(); p_idx++)
		    {
			if(op_ptr->adds().isset(pw->at(p_idx)) || op_ptr->dels().isset(pw->at(p_idx)) || m_task.fast_op_edeletes(o_idx).isset( pw->at(p_idx) ) )
			  {
			    useable[o_idx]=false;
			    break;
			  }
		    }
		}
	  }
	

	unsigned i = 0;
	for ( unsigned k = 0; k < s->atom_vec().size(); k++ )
	{
		unsigned p = s->atom_vec()[k];
		P.set( p );	
		value( p ) = std::min( i, value(p) );
	}

	if ( can_apply( P, m_task.end() ) )
	{
		value_op( m_task.end() ) = i;
		return;
	}
	for ( unsigned op_idx = 2; op_idx < m_task.useful_ops().size(); op_idx++ )
	{
		if ( !can_apply(P, op_idx) ) continue;
		if ( !consistent( i, op_idx ) ) continue;
		if(!useable[op_idx])	  continue;
		A.push_back( op_idx );
		value_op( op_idx ) = i;
	}

	while ( true )
	{
		i++;
		bool new_atoms = false;
		for ( unsigned k = 0; k < A.size(); k++ )
		{
			PDDL::Operator* a = m_task.useful_ops()[A[k]];
			Atom_Vec& adds = a->add_vec();
			for ( unsigned j = 0; j < adds.size(); j++ )
			{
				if ( P.isset( adds[j] ) ) continue;
				P.set( adds[j] );
				value( adds[j] ) = std::min( value(adds[j]), i );
				new_atoms = true;
			}
		}

		
		if ( can_apply( P, m_task.end() ) )
		{
			value_op( m_task.end() ) = i;
			return;
		}
		else
		{
			if (!new_atoms ) return;
		}
		A.clear();
		for ( unsigned op_idx = 2; op_idx < m_task.useful_ops().size(); op_idx++ )
		{
			if ( !can_apply(P, op_idx) ) continue;
			if ( !consistent( i, op_idx ) ) continue;
			if(!useable[op_idx])	  continue;
			A.push_back( op_idx );
			value_op( op_idx ) = std::min( i, value_op(op_idx) );
		}
		
	}
}


}
