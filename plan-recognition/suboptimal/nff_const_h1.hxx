#ifndef __NFF_Constrained_H1__
#define __NFF_Constrained_H1__

#include "ext_math.hxx"
#include "PDDL.hxx"
#include "nff.hxx"
#include "nff_state.hxx"

namespace NFF
{

class Supports_Table;


class Constrained_H1
{
public:

	struct Layer
	{
		Atom_Vec 		Goals;
		PDDL::Fluent_Set*	Marked_True;
	};


	Constrained_H1();
	~Constrained_H1();
	
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

	unsigned& value_op( unsigned op )
	{
		return m_op_values[op];
	}

	unsigned operator()( State* s )
	{
		return eval( s->atom_vec() );
	}

	template <typename Node_Type>
	void compute( Node_Type* n )
	{
		compute( n->s, *(n->Locked), n->Pending );
	}
	
	void compute( State* s, PDDL::Fluent_Set& protected_atoms, Support_List& pending );

  	void compute_with_persist( State* s, PDDL::Fluent_Set& protected_atoms, Support_List& pending, Atom_Vec* pw);
      
    	template <typename Node_Type>
	void compute_with_persist( Node_Type* n, Atom_Vec* pw )
	{
      		compute_with_persist( n->s, *(n->Locked), n->Pending, pw );
	}

	void compute_difficulties( );

	void collect_constraints( State* s, PDDL::Fluent_Set& protected_atoms, Support_List& pending );

	void set_goal( unsigned op ) { m_end_op = op; }

protected:

	bool can_apply( PDDL::Fluent_Set& P, unsigned op );
	bool consistent( unsigned i, unsigned op );
	
protected:
	PDDL::Task&				m_task;
	std::vector<unsigned>			m_values;
	std::vector<unsigned>			m_op_values;
	unsigned				m_action_cost;
	unsigned				m_Nf;
	std::vector<Operator_Set>		m_constraints;
	std::vector<unsigned>			m_difficulties;
	unsigned				m_end_op;

};

}

#endif // nff_h1.hxx
