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
#ifndef __NFF_LAZY_ABSTRACT_PROPAGATOR__
#define __NFF_LAZY_ABSTRACT_PROPAGATOR__

#include "nff_options.hxx"
#include "nff_prop_util.hxx"
#include "nff_dfs_node.hxx"
#include "nff_h2.hxx"

namespace NFF
{

template <typename Heuristic>
class Lazy_Abstract_Propagator
{
protected:
	Atom_Vec			m_scratch;
	PDDL::Task&			m_task;
public:

	PDDL::Task&			task();
	Propagation_Graph		Graph;
	std::vector<Index_Vector>	Atom_Layers;
	std::vector<Index_Vector>	Op_Layers;

	typedef				Heuristic	Heuristic_Type;
	Heuristic			H;
		
	unsigned			Max_Label_Size;

	Lazy_Abstract_Propagator()
		: m_task( PDDL::Task::instance() ), Max_Label_Size( 0 ), Current_Min_Action( 0 ), m_goal_op( m_task.end() ),
		Current_Prec( 0 )
	{
		m_scratch.reserve( task().fluents().size() );
		Graph.reserve( task().fluents().size() + task().useful_ops().size() + 2);
	}
	
	virtual ~Lazy_Abstract_Propagator()
	{
		for ( unsigned k = 0; k < Graph.size(); k++ )
			delete Graph[k];
	}

	void				set_goal( unsigned op ) { m_goal_op = op; }

	unsigned			build_propagation_graph( Dfs_Node* n );
	void				get_applicable_op( Operator_Vec& min_action_set );
	void				get_applicable_op( Operator_Vec& min_action_set, Dfs_Node* n );
	virtual bool			find_first_ccc( Dfs_Node* n, unsigned min_action ) = 0;
	virtual bool			find_next_ccc(Dfs_Node* n) = 0;

	unsigned			Current_Min_Action;
	unsigned			m_goal_op;
	unsigned			Current_Prec;
	Causal_Chain			Current_Chain;
	Propagation_Trace		Trace;

protected:
	void				make_goal_nodes( std::map<unsigned, unsigned> & atom_indices);
	bool				structural_mutex( Ext_State& s );
	std::map<unsigned, unsigned> 	atom_indices;
	std::map<unsigned, unsigned> 	operator_indices;
	
};

template <typename Heuristic>
inline PDDL::Task&	Lazy_Abstract_Propagator<Heuristic>::task()
{
	return m_task;
}

template <typename Heuristic>
void	Lazy_Abstract_Propagator<Heuristic>::get_applicable_op( Operator_Vec& min_action_set )
{
	for ( unsigned j = 0; j < Op_Layers[0].size(); j++ )
		min_action_set.push_back( Graph[ Op_Layers[0][j] ]->Operator );
}

template <typename Heuristic>
void	Lazy_Abstract_Propagator<Heuristic>::get_applicable_op( Operator_Vec& min_action_set, Dfs_Node* n )
{
	for ( unsigned j = 0; j < Op_Layers[0].size(); j++ )
	{
		unsigned op = Graph[ Op_Layers[0][j] ]->Operator;
		PDDL::Operator* op_ptr = task().useful_ops()[ op ];
		Atom_Vec& op_adds =  op_ptr->add_vec();
		bool pruned = false;

		for ( unsigned k = 0; k < op_adds.size() && !pruned; k++ )
		{
	      		if ( n->Locked->isset( op_adds[k] )  )
			{	
      				for ( Support_List::iterator j = n->Pending.begin(); j != n->Pending.end(); j++ )
	    				if ( j->p == op_adds[k] && !m_task.equal_effects(j->a, op) )
					{		
				  
#ifndef NDEBUG
			      			std::cout << "Atom Added is locked: "; 
				      		m_task.print_fluent( m_task.fluents()[op_adds[k]], std::cout );
#endif
						pruned = true;
						break;
					}
			}
		}

		Atom_Vec& op_dels = op_ptr->del_vec();

		for ( unsigned k = 0;  k < op_dels.size() && !pruned ; k++ )
		{
			if ( n->Locked->isset( op_dels[k] ) )
			{
				for ( Support_List::iterator j = n->Pending.begin(); j != n->Pending.end(); j++ )
					if ( j->p == op_dels[k] && !m_task.equal_effects(j->a, op) )
					{	
				
#ifndef NDEBUG
						std::cout << "Atom Del is in pending: "; 
						m_task.print_fluent( m_task.fluents()[op_dels[k]], std::cout );
						std::cout << "for Op: ";
						m_task.print_operator( m_task.useful_ops()[op], std::cout );
						std::cout << std::endl;
#endif

						pruned = true;
						break;
					}
			}
		}
		if (!pruned) min_action_set.push_back( op );
	}
}

template <typename Heuristic>
unsigned Lazy_Abstract_Propagator<Heuristic>::build_propagation_graph( Dfs_Node* n )
{
	H.set_goal( m_goal_op );
	H.compute(n);
	unsigned hmax_s = H.value_op( m_goal_op );
	if ( hmax_s == std::numeric_limits<unsigned>::max() )
		return hmax_s;

	Atom_Layers.resize( (unsigned)hmax_s +2 ); // leave room for dummy
	Op_Layers.resize( (unsigned)hmax_s + 1 ); // leave room for End

	make_goal_nodes( atom_indices );

	for ( unsigned i = (unsigned)hmax_s; i > 0; i-- )	
	{
		for ( unsigned j = 0; j < Atom_Layers[i].size(); j++ )
		{
			Propagator_Node* n = Graph.at( Atom_Layers[i][j] );
			Operator_Vec& supporters = task().added_by( n->Atom );
			bool at_least_one_supporter = false;
			for ( unsigned k = 0; k < supporters.size(); k++ )
			{

				if ( H.value_op(supporters[k]) == i - task().op_cost(supporters[k])  ) 
				{
					unsigned a_p_idx = operator_indices[ supporters[k] ];
					at_least_one_supporter = true;
					if ( a_p_idx == 0 )
					{
						Propagator_Node* n2 = new Propagator_Node;
						n2->Type = OPERATOR;
						n2->Operator = supporters[k];
						Graph.push_back( n2 );
						Op_Layers.at((unsigned)H.value_op(supporters[k])).push_back( Graph.size() - 1 );
						n->Parents.push_back( Graph.size() - 1 );
						n2->Children.push_back( Atom_Layers[i][j] );
						operator_indices[ supporters[k] ] = Graph.size() - 1;
					}	
					else
					{
						n->Parents.push_back( a_p_idx );
						Propagator_Node* n2 = Graph[a_p_idx];
						n2->Children.push_back( Atom_Layers[i][j] );
					}
				}
			}
			if ( !at_least_one_supporter )
			{
				std::cout << i << std::endl;
			}
			assert( at_least_one_supporter );
		}

		for ( unsigned j = 0; j < Op_Layers[i-1].size(); j++ )
		{
			Propagator_Node* n = Graph.at(Op_Layers[i-1][j]);
			PDDL::Operator* op_ptr = task().useful_ops()[ n->Operator ];
			Atom_Vec& preconditions = op_ptr->prec_vec();
			for ( unsigned k = 0; k < preconditions.size(); k++ )
			{
				unsigned prec_idx = atom_indices[ preconditions[k] ];
				if ( prec_idx == 0 )
				{
					Propagator_Node* n2 = new Propagator_Node;
					n2->Type = ATOM;
					n2->Atom = preconditions[k];
					Graph.push_back( n2 );
					Atom_Layers.at((unsigned)H.value(n2->Atom)).push_back(Graph.size()-1);
					n->Parents.push_back( Graph.size()-1 );
					n2->Children.push_back( Op_Layers[i-1][j] );
					atom_indices[ preconditions[k] ] = Graph.size()-1;
				}
				else
				{
					n->Parents.push_back( prec_idx );
					Propagator_Node* n2 = Graph[prec_idx];
					n2->Children.push_back( Op_Layers[i-1][j] );
				}
			}
		}
	}

	#ifndef NDEBUG
	/*
	for ( int i = hmax_s; i > 0; i-- )
	{
		std::cout << "Proposition Layer #" << i << ": ";
		for ( unsigned j = 0; j < Atom_Layers[i].size(); j++ )
		{
			task().print_fluent( task().fluents()[ Graph[Atom_Layers[i][j]]->Atom ], std::cout );
			if ( j < Atom_Layers[i].size()-1 )
				std::cout << ", ";
		}
		std::cout << std::endl;

		std::cout << "Action Layer #" << i-1 << ": ";
		for ( unsigned j = 0; j < Op_Layers[i-1].size(); j++ )
		{
			task().print_operator( task().useful_ops()[ Graph[Op_Layers[i-1][j]]->Operator ], std::cout );
			if ( j < Op_Layers[i-1].size()-1 )
				std::cout << ", ";
		}
		std::cout << std::endl;
	}
	std::cout << "Proposition Layer #" << 0 << ": ";
	for ( unsigned j = 0; j < Atom_Layers[0].size(); j++ )
	{
		task().print_fluent( task().fluents()[ Graph[Atom_Layers[0][j]]->Atom ], std::cout );
		if ( j < Atom_Layers[0].size()-1 )
			std::cout << ", ";
	}
	std::cout << std::endl;
	*/
	#endif
	return hmax_s;
}

template <typename Heuristic>
bool Lazy_Abstract_Propagator<Heuristic>::structural_mutex( Ext_State& s )
{
	m_scratch.clear();
	for ( unsigned p = 1; p < task().fluents().size(); p++ )
		if ( s[p] == l_True ) m_scratch.push_back( p );
	unsigned v = task().h2().eval( m_scratch );

	return v == std::numeric_limits<unsigned>::max();
}

template <typename Heuristic>
void Lazy_Abstract_Propagator<Heuristic>::make_goal_nodes( std::map<unsigned, unsigned> & atom_indices)
{
	// Dummy goal and End() layer
	Propagator_Node* n = new Propagator_Node;
	n->Type = ATOM;
	n->Atom = 0;
	n->Parents.push_back( 1 );
	Graph.push_back( n );
	Atom_Layers.back().push_back( 0 );	

	Propagator_Node* end_op_node = new Propagator_Node;
	end_op_node->Type = OPERATOR;
	end_op_node->Operator = m_goal_op;
	//end_op_node->Children.push_back( 0 );
	Graph.push_back( end_op_node );
	Op_Layers.back().push_back( Graph.size() - 1);
	
	PDDL::Operator* end_op = task().useful_ops()[ m_goal_op ];

	for ( unsigned k = 0; k < end_op->prec_vec().size(); k++ )
	{
		Propagator_Node* n = new Propagator_Node;
		n->Type = ATOM;
		n->Atom = end_op->prec_vec()[k];
		//n->Children.push_back( 1 );
		Graph.push_back( n );
		end_op_node->Parents.push_back( Graph.size()-1 );
		n->Children.push_back( 1 );
		atom_indices[ n->Atom ] = Graph.size()-1;
		Atom_Layers[ (unsigned)H.value(n->Atom) ].push_back( Graph.size() -1 );
	}
}

}

#endif // nff_abs_prop.hxx
