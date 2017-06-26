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
#ifndef __NFF_LAZY_JP_PROPAGATOR__
#define __NFF_LAZY_JP_PROPAGATOR__

#include "nff_prop_util.hxx"
#include "nff_lazy_abs_prop.hxx"
#include "nff_h2.hxx"

namespace NFF
{

template <typename Heuristic>
class C3_Lazy_JP_Propagator : public Lazy_Abstract_Propagator<Heuristic>
{
public:
	C3_Lazy_JP_Propagator()
	{
	}
	
	virtual ~C3_Lazy_JP_Propagator()
	{
	}

	bool				find_first_ccc( Dfs_Node* n, unsigned min_action );
	bool				find_next_ccc( Dfs_Node* n );
	bool				propagate( Propagator_Node* n );
	void				extract_ccc_from_trace();
	bool				can_continue_with( Causal_Chain& c );
	bool				can_continue_with( Support_List::iterator it, Support_List::iterator end_it, Ext_State* label );

protected:

	void				enforce_joint_persistency( Ext_State& L, unsigned p );
};

template <typename Heuristic>
void C3_Lazy_JP_Propagator<Heuristic>::extract_ccc_from_trace()
{

	typedef Lazy_Abstract_Propagator<Heuristic> B;

	B::Current_Chain.body.clear();
	Support sup;
	sup.p = 0;
	sup.a = B::m_goal_op;
	B::Current_Chain.body.push_front( sup );

	for ( Propagation_Trace::reverse_iterator it = B::Trace.rbegin();
		it != B::Trace.rend() && (*it)->node->Operator != (int)B::Current_Min_Action; 
		it++ )
	{
		if ( (*it)->node->Type == ATOM )
			B::Current_Chain.body.front().p = (*it)->node->Atom;
		else
		{
			Support sup;
			sup.p = 0;
			sup.a = (*it)->node->Operator;
			B::Current_Chain.body.push_front( sup );
		}
	}
}

template <typename Heuristic>
bool C3_Lazy_JP_Propagator<Heuristic>::find_first_ccc( Dfs_Node* search_node, unsigned min_action )
{
	typedef Lazy_Abstract_Propagator<Heuristic> B;

	assert( min_action != B::Current_Min_Action );
	if ( min_action != B::Current_Min_Action )
	{
		B::Current_Min_Action = min_action;
		B::Current_Prec = 0;
		B::Current_Chain.a0 = B::Current_Min_Action;
		B::Current_Chain.body.clear();
		for ( Propagation_Trace::iterator it = B::Trace.begin(); 
			it != B::Trace.end(); it++ )
			delete (*it)->out_label; // Only delete labels info, nodes aren't his to delete
		B::Trace.clear();
	}	

	Propagator_Trace_Node* initial = new Propagator_Trace_Node;
	initial->node = NULL;
	initial->index = 0;
	initial->out_label = new Ext_State( search_node->s->atom_vec() );
	B::Trace.push_back( initial );
	//#ifndef NDEBUG
	//std::cout << "L-(a0) = ";
	//initial->out_label->print();	
	//#endif
	if ( propagate( B::Graph[ B::operator_indices[ B::Current_Min_Action ] ] ) )
	{
		extract_ccc_from_trace();
		return true;
	}
	
	return false;
}

template <typename Heuristic>
bool C3_Lazy_JP_Propagator<Heuristic>::find_next_ccc( Dfs_Node* n )
{
	typedef Lazy_Abstract_Propagator<Heuristic> B;

	bool new_chain_found = false;
	
	while ( !B::Trace.empty() && !new_chain_found )
	{
		Propagator_Trace_Node* tn = B::Trace.back();
		if ( tn->node == NULL || tn->index == tn->node->Children.size()-1 )
		{
			delete tn->out_label;
			delete tn;
			B::Trace.pop_back();
			continue;
		}
		for ( unsigned k = tn->index+1; k < tn->node->Children.size(); k++ )
		{
			tn->index = k;
			if ( propagate( B::Graph[ tn->node->Children[k] ] ) )
			{
				new_chain_found = true;
				break;
			}
		}
		if ( !new_chain_found )
		{
			delete tn->out_label;
			delete tn;
			B::Trace.pop_back();
		}
	}

	if ( new_chain_found )
	{
		extract_ccc_from_trace();
		return true;
	}
	
	return false;	
}

template <typename Heuristic>
bool C3_Lazy_JP_Propagator<Heuristic>::propagate( Propagator_Node* n )
{
	typedef Lazy_Abstract_Propagator<Heuristic> B;

	Propagator_Trace_Node* prev_ctx = B::Trace.back();
	if ( n->Type == ATOM )
	{
		//#ifndef NDEBUG
		//std::cout << "Atom: ";
		//B::task().print_fluent( B::task().fluents()[n->Atom], std::cout );
		//std::cout << std::endl;
		//#endif
		Ext_State* out_label = new Ext_State( *(prev_ctx->out_label) );
		enforce_joint_persistency( *out_label, n->Atom );
		//#ifndef NDEBUG
		//std::cout << "L-(ai) = "; out_label->print();
		//#endif
		Propagator_Trace_Node* current_ctx = new Propagator_Trace_Node;
		current_ctx->node = n;
		current_ctx->out_label = out_label;
		B::Trace.push_back( current_ctx );
		for ( unsigned k = 0; k < n->Children.size(); k++ )
		{
			current_ctx->index = k;
			if ( propagate( B::Graph[ n->Children[k] ] ) )
				return true;
		}	

		delete current_ctx->out_label;
		B::Trace.pop_back();
		return false;
	}
	//#ifndef NDEBUG
	//std::cout << "Operator: ";
	//B::task().print_operator( B::task().useful_ops()[n->Operator], std::cout );
	//std::cout << std::endl;
	//#endif

	PDDL::Operator* op_ptr = B::task().useful_ops()[n->Operator];
	Atom_Vec& prec = op_ptr->prec_vec();
	for ( unsigned i = 0; i < prec.size(); i++ )
		if ( (*(prev_ctx->out_label))[prec[i]] == l_False )
		{
			//#ifndef NDEBUG
			//std::cout << "Inconsistent with precondition of action ai!" << std::endl;
			//#endif
			return false;
		}
	Ext_State* out_label = new Ext_State( *(prev_ctx->out_label) );						
	out_label->assign( prec, l_True );
	if ( B::structural_mutex( *out_label ) )
	{
		//#ifndef NDEBUG
		//std::cout << "Structural Mutex with precondition of action ai!" << std::endl;
		//#endif

		delete out_label;
		return false;
	}

	if ( (unsigned)n->Operator == B::m_goal_op )
	{
		delete out_label;
		return true;
	}

	// restore truth values
	for ( unsigned k = 0; k < prec.size(); k++ )
		(*out_label)[prec[k]] = (*(prev_ctx->out_label))[prec[k]];

	out_label->update( n->Operator );
	//#ifndef NDEBUG
	//std::cout << "L+(ai) = "; out_label->print( );
	//#endif

	
	Propagator_Trace_Node* current_ctx = new Propagator_Trace_Node;
	current_ctx->node = n;
	current_ctx->out_label = out_label;
	B::Trace.push_back( current_ctx );
	for ( unsigned k = 0; k < n->Children.size(); k++ )
	{
		current_ctx->index = k;
		if ( propagate( B::Graph[ n->Children[k] ] ) )
			return true;
	}	

	delete current_ctx->out_label;
	B::Trace.pop_back();
	return false;
}

template <typename Heuristic>
void C3_Lazy_JP_Propagator<Heuristic>::enforce_joint_persistency( Ext_State& L, unsigned p )
{
	typedef Lazy_Abstract_Propagator<Heuristic> B;

	bool changed;
	do
	{
		changed = false;
		for ( unsigned i = 1; i < B::task().fluents().size(); i++ )
		{
			if ( L[i] == l_Undef ) continue; // not in label
			unsigned q = i;
			bool in_J = false;
			Operator_Vec& ops_q = ( L[i] == l_False ? B::task().added_by(i) : B::task().deleted_by(i) );
			for ( unsigned j = 0; j < ops_q.size(); j++ )
			{
				//if ( B::H.value_op(ops_q[j]) == std::numeric_limits<unsigned>::max() ) continue;
				if ( !B::task().reachable(ops_q[j])) continue;
				PDDL::Operator* op = B::task().useful_ops()[ops_q[j]];
				
				if (  !op->adds().isset(p) && !op->dels().isset(p) && !B::task().fast_op_edeletes(ops_q[j]).isset(p) )
				{
					in_J = true;
					for ( unsigned k = 0; k < op->prec_vec().size(); k++ )
					{
						if ( L[ op->prec_vec()[k] ] == l_False )
						{
							in_J = false;
							break;
						}
					}
					if ( in_J )
					{
						std::vector<lbool> prev_vals;
						prev_vals.resize( op->prec_vec().size() );
						for ( unsigned k = 0; k < op->prec_vec().size(); k++ )
							prev_vals[k] = L[op->prec_vec()[k]];
						L.assign( op->prec_vec(), l_True );
						if ( B::structural_mutex( L ) )
							in_J = false;
						for ( unsigned k = 0; k < prev_vals.size(); k++ )
							L[ op->prec_vec()[k] ] = prev_vals[k];
					}
				}		
				if ( in_J ) 
				{
					L[q] = l_Undef;
					changed = true;
					break;	
				}	
			}
		}
	} while ( changed );	
}

template <typename Heuristic>
bool  C3_Lazy_JP_Propagator<Heuristic>::can_continue_with( Causal_Chain& c )
{
	typedef Lazy_Abstract_Propagator<Heuristic> B;

	Ext_State* a0_m_label = B::Trace.back()->out_label;
	//#ifndef NDEBUG
	//std::cout << "L-(ai) = "; a0_m_label->print();
	//#endif

	PDDL::Operator* op_ptr = B::task().useful_ops()[c.a0];
	//#ifndef NDEBUG
	//std::cout << "Operator: ";
	//B::task().print_operator( B::task().useful_ops()[c.a0], std::cout );
	//std::cout << std::endl;
	//#endif

	assert( op_ptr->preconds().isset( B::Trace.back()->node->Atom ) );

	Atom_Vec& prec = op_ptr->prec_vec();
	for ( unsigned i = 0; i < prec.size(); i++ )
		if ( (*(a0_m_label))[prec[i]] == l_False )
		{
			//#ifndef NDEBUG
			//std::cout << "Inconsistent with precondition of action ";
			//B::task().print_operator( op_ptr, std::cout );
			//std::cout << " because of fluent ";
			//B::task().print_fluent( B::task().fluents()[ prec[i] ], std::cout );
			//std::cout << std::endl;
			//#endif
			return false;
		}
	Ext_State* out_label = new Ext_State( *(a0_m_label) );						
	out_label->assign( prec, l_True );
	if ( B::structural_mutex( *out_label ) )
	{
		//#ifndef NDEBUG
		//std::cout << "Structural Mutex with precondition of action " << std::endl;
		//B::task().print_operator( op_ptr, std::cout );
		//std::cout << std::endl;
		//#endif

		delete out_label;
		return false;
	}
	// restore truth values
	for ( unsigned k = 0; k < prec.size(); k++ )
		(*out_label)[prec[k]] = (*(a0_m_label))[prec[k]];

	out_label->update( c.a0 );
	//#ifndef NDEBUG
	//std::cout << "L+(ai) = "; out_label->print( );
	//#endif

	bool can_continue = can_continue_with( c.body.begin(), c.body.end(), out_label );

	delete out_label;

	return can_continue;
}

template <typename Heuristic>
bool  C3_Lazy_JP_Propagator<Heuristic>::can_continue_with( Support_List::iterator it, Support_List::iterator end_it, Ext_State* label )
{
	typedef Lazy_Abstract_Propagator<Heuristic> B;

	if ( it == end_it ) return true;
	//#ifndef NDEBUG
	//std::cout << "Atom: ";
	//B::task().print_fluent( B::task().fluents()[it->p], std::cout );
	//std::cout << std::endl;
	//#endif

	Ext_State* p_label = new Ext_State( *(label) );
	enforce_joint_persistency( *p_label, it->p );
	//#ifndef NDEBUG
	//std::cout << "L-(ai) = "; p_label->print( );
	//#endif

	//#ifndef NDEBUG
	//std::cout << "Operator: ";
	//B::task().print_operator( B::task().useful_ops()[it->a], std::cout );
	//std::cout << std::endl;
	//#endif

	PDDL::Operator* op_ptr = B::task().useful_ops()[it->a];
	Atom_Vec& prec = op_ptr->prec_vec();
	for ( unsigned i = 0; i < prec.size(); i++ )
		if ( (*(p_label))[prec[i]] == l_False )
		{
			//#ifndef NDEBUG
			//std::cout << "Inconsistent with precondition of action ";
			//B::task().print_operator( op_ptr, std::cout );
			//std::cout << std::endl;
			//std::cout << " because of fluent ";
			//B::task().print_fluent( B::task().fluents()[ prec[i] ], std::cout );
			//std::cout << std::endl;
			//#endif

			return false;
		}

	Ext_State* out_label = new Ext_State( *(p_label) );						
	out_label->assign( prec, l_True );
	if ( B::structural_mutex( *out_label ) )
	{
		//#ifndef NDEBUG
		//std::cout << "Structural Mutex with precondition of action " << std::endl;
		//B::task().print_operator( op_ptr, std::cout );
		//std::cout << std::endl;
		//#endif

		delete out_label;
		return false;
	}

	// restore truth values
	for ( unsigned k = 0; k < prec.size(); k++ )
		(*out_label)[prec[k]] = (*(p_label))[prec[k]];

	out_label->update( it->a );
	//#ifndef NDEBUG
	//std::cout << "L+(ai) = "; out_label->print( );
	//#endif
	Support_List::iterator next_it = it;
	std::advance( next_it, 1 );
	bool can_continue = can_continue_with( next_it, end_it, out_label );

	delete p_label;
	delete out_label;

	return can_continue;
}

}

#endif // nff_propagator.hxx
