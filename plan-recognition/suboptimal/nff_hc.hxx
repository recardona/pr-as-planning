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
#ifndef __NFF_HC__
#define __NFF_HC__

#include "nff_dfs_node.hxx"
#include "nff.hxx"
#include "PDDL.hxx"
#include "pddl_fluent_table.hxx"
#include "nff_algobase.hxx"
#include "nff_options.hxx"
#include "nff_h_add_plus.hxx"

namespace NFF
{
template <typename Propagator_Type>
class Hill_Climbing : public DFS_Skeleton
{
public:

	typedef  typename Propagator_Type::Heuristic_Type	Heuristic;

	Hill_Climbing()
		: m_task( PDDL::Task::instance() ), m_closed( 8192 )
	{
		m_c2_scores.resize( m_task.useful_ops().size() );
		LH.compute_classic_hadd();
	}

	~Hill_Climbing()
	{
	}

	bool solve();

protected:
	void 		close( Dfs_Node* n );
	Dfs_Node* 	closed( Dfs_Node* n );
	void		remove( Dfs_Node* n );
	void		top_level_search();
	Dfs_Node*	do_search( Dfs_Node* n, Causal_Chain& tail );
	void		handle_solution( Dfs_Node* G );
	void		write_plan();

protected:
	typedef	PDDL::Fluent_Set_Hashtable<Dfs_Node>				Closed_List;
	PDDL::Task&								m_task;
	State*									goal;
	Closed_List								m_closed;
	std::vector<float>							m_c2_scores;
	Additive_Heuristic_Plus							LH;
	unsigned 								Last_Op;
	float									Last_H;
	/*
	unsigned								Best_Plan_Cost;
	unsigned								Plan_Index;
	*/
};

template <typename Propagator_Type>
bool Hill_Climbing<Propagator_Type>::solve()
{
	Dfs_Node* n = Dfs_Node::root();
	goal = State::make_goal_state();
	Dfs_Node* G = NULL;
	Last_Op = 0;
	Last_H = std::numeric_limits<float>::max();
	/*
	Best_Plan_Cost = std::numeric_limits<unsigned>::max();
	Plan_Index = 0;
	top_level_search( );	
	*/
	Causal_Chain initial_tail;
	initial_tail.a0 = m_task.end();
	G = do_search( n, initial_tail );
	if ( G!= NULL )
	{
		std::vector<unsigned> p;
		Dfs_Node* n2 = G;
		while ( n2->parent != NULL )
		{
			p.push_back( n2->op );
			n2 = n2->parent;
		}
		for ( int k = p.size()-1; k >= 0; k-- )
			path_found.push_back( p[k] );
	}
	m_closed.clear();	
	return G != NULL;

	//return Plan_Index != 0;
}
/*
template <typename Propagator_Type>
void	Hill_Climbing<Propagator_Type>::write_plan()
{
	char out_plan_fname[512];
	snprintf( out_plan_fname, 511, "plan.soln.%d", Plan_Index );
	std::ofstream plan_out( out_plan_fname );
	unsigned action_count = 1;
	for ( unsigned k = 0; k < path_found.size(); k++ )
	{
		PDDL::Operator* op = m_task.useful_ops()[path_found[k]];
		plan_out << "[" << action_count++ << "] ";
		m_task.print_operator( op, plan_out );
		plan_out << std::endl;
	}
	plan_out.close();
}

template <typename Propagator_Type>
void	Hill_Climbing<Propagator_Type>::handle_solution( Dfs_Node* G )
{
	path_found.clear();
	std::vector<unsigned> p;
	Dfs_Node* n2 = G;
	while ( n2->parent != NULL )
	{
		p.push_back( n2->op );
		n2 = n2->parent;
	}
	for ( int k = p.size()-1; k >= 0; k-- )
		path_found.push_back( p[k] );
	Plan_Index++;
	write_plan();
}

template <typename Propagator_Type>
void	Hill_Climbing<Propagator_Type>::top_level_search( )
{
	Dfs_Node* n = Dfs_Node::root();
	Propagator_Type* prop = new Propagator_Type;

	if ( prop->build_propagation_graph( n ) == std::numeric_limits<unsigned>::max() )
	{
		#ifndef NDEBUG
		std::cout << "At top-level: Dead-End" << std::endl;
		#endif

		delete prop;
		delete n;
		return;
	}
	
	Operator_Vec R;
	prop->get_applicable_op(R);

	if ( R.empty() )
	{
		#ifndef NDEBUG
		std::cout << "At top level: No consistent minimal actions were found" << std::endl;
		#endif
		delete n;
		delete prop;
		return; // dead--end
	}

	if ( R.size() > 1 )
	{
		for ( unsigned k = 0; k < R.size(); k++ )
		{
			Dfs_Node* n2 = n->successor( R[k] );
			LH.compute(n2);
			float h = LH.eval_ff( goal->atom_vec() );
			m_c2_scores[R[k]] = std::add( h, n2->gn );
			delete n2;	
		}

		std::sort( R.begin(), R.end(), Sort_By_Score(m_c2_scores) );
	}

	#ifndef NDEBUG
	std::cout << "At top-level: ";
	std::cout << " Minimal applicable actions scoring: " << std::endl;
	for ( unsigned i = 0; i < R.size(); i++ )
	{
		std::cout << "\t";
		m_task.print_operator( m_task.useful_ops()[ R[i] ], std::cout );
		std::cout << " score: " << m_c2_scores[R[i]] << std::endl;
	}
	#endif
	
	for (unsigned i = 0; i < R.size(); i++ )
	{
		if ( R[i] == m_task.start() ||  m_c2_scores[R[i]] == std::numeric_limits<unsigned>::max()  ) continue;
		#ifndef NDEBUG
		std::cout << "At top-level: Considering action: ";
		m_task.print_operator( m_task.useful_ops()[ R[i]], std::cout );
		std::cout << std::endl;
		#endif
		if ( prop->find_first_ccc(n, R[i])  )
		{
			#ifndef NDEBUG
			std::cout << "At top-level: Causal Chain found: " << std::endl;
			prop->Current_Chain.print(std::cout);
			std::cout << std::endl;
			#endif
			Causal_Chain New_Tail;
			New_Tail.a0 = m_task.end();
			#ifndef NDEBUG
			std::cout << "At top-level: applying action: ";
			m_task.print_operator( m_task.useful_ops()[R[i]], std::cout );
			std::cout << std::endl;	
			#endif
			Dfs_Node* ni = Dfs_Node::root();
			close(ni);
			Dfs_Node* c = ni->successor( R[i] );
			Dfs_Node* r = do_search( c, New_Tail );
			if ( r != NULL ) 
			{
				handle_solution(r);
				Best_Plan_Cost = r->gn;
				std::cout << "HC: Plan found of cost: " << Best_Plan_Cost << std::endl;
			}
			m_closed.clear();

			while( prop->find_next_ccc( n ) )
			{
				#ifndef NDEBUG
				std::cout << "At top-level: Causal Chain found: " << std::endl;
				prop->Current_Chain.print(std::cout);
				std::cout << std::endl;
				#endif
				Causal_Chain New_Tail;
				New_Tail.a0 = m_task.end();
				#ifndef NDEBUG
				std::cout << "At top-level: applying action: ";
				m_task.print_operator( m_task.useful_ops()[R[i]], std::cout );
				std::cout << std::endl;	
				#endif
				Dfs_Node* ni = Dfs_Node::root();
				close(ni);
				Dfs_Node* c = ni->successor( R[i] );
				Dfs_Node* r = do_search( c, New_Tail );
				if ( r != NULL ) 
				{
					handle_solution(r);
					Best_Plan_Cost = r->gn;
					std::cout << "HC: Plan found of cost: " << Best_Plan_Cost << std::endl;
				}
				m_closed.clear();
			}	
		}
	}

	delete prop;
	delete n;
}
*/
template <typename Propagator_Type>
Dfs_Node*	Hill_Climbing<Propagator_Type>::do_search( Dfs_Node* n, Causal_Chain& Tail )
{
	/*
	if ( n->gn >= Best_Plan_Cost )
		return NULL; // We can't find a better plan 
	*/
	if ( n->s->can_apply( m_task.end() ) ) 
		return n; // Solution found
	#ifndef NDEBUG
	std::cout << "Depth: " << n->gn << std::endl;
	std::cout << "State: ";
	n->s->print(std::cout );
	std::cout << std::endl;
	#endif

	// Progress
	#ifndef NDEBUG
	std::cout << "Current Tail: ";
	Tail.print(std::cout);
	std::cout << std::endl;
	#endif
	if ( n->s->can_apply( Tail.a0 ) )
	{
		#ifndef NDEBUG
		std::cout << "Next action in tail: ";
		m_task.print_operator( m_task.useful_ops()[Tail.a0], std::cout );
		std::cout << " is applicable, proceeding with current chain" << std::endl;
		#endif
		Dfs_Node* c = n->successor( Tail.a0 );
		if ( closed(c) ) return NULL;
		close(n);
		Tail.progress();
		Dfs_Node* r = do_search( c, Tail );
		if ( r != NULL )
			return r;
		return NULL; 
	}
	#ifndef NDEBUG
	std::cout << "Checking progress... ";
	std::cout.flush();
	#endif
	LH.compute(n);
	if ( Last_Op != Tail.a0 )
	{
		#ifndef NDEBUG
		std::cout << "a0 of Tail changed" << std::endl;
		#endif
		Last_Op = Tail.a0;
		PDDL::Operator* op_ptr = m_task.useful_ops()[Tail.a0];
		Atom_Vec& precs = op_ptr->prec_vec();
		Last_H = LH.metric_eval( precs );
		if ( Last_H == std::numeric_limits<float>::max() )
			return NULL;
	}
	else
	{
		PDDL::Operator* op_ptr = m_task.useful_ops()[Tail.a0];
		Atom_Vec& precs = op_ptr->prec_vec();
		float new_H = LH.metric_eval( precs );
		if ( new_H >= Last_H )
		{
			#ifndef NDEBUG
			std::cout << "No progress made, resetting Tail to END()" << std::endl;
			std::cout << "Last: " << Last_H << " New: " << new_H << std::endl;
			#endif
			Tail.body.clear();
			Tail.a0 = m_task.end();
			Last_H = std::numeric_limits<float>::max();
		}
		else
		{
			#ifndef NDEBUG
			std::cout << "Progress made, from " << Last_H << " to " << new_H << std::endl;
			#endif
			Last_H = new_H;
		}
	}

	#ifndef NDEBUG
	std::cout << "Tail size: " << Tail.body.size() << std::endl;
	#endif

	Propagator_Type* prop = new Propagator_Type;
	prop->set_goal( Tail.a0 );

	if ( prop->build_propagation_graph( n ) == std::numeric_limits<float>::max() )
	{

		Tail.body.clear();
		Tail.a0 = m_task.end();
		Dfs_Node* r = do_search( n, Tail );
		if ( r != NULL ) return r;
		delete prop;
		delete n;
		return NULL;
	}
	
	Operator_Vec								R;
	evaluated++;
	max_label_size = 0;
	avg_label_size = 0;
	prop->get_applicable_op(R);

	if ( R.empty() )
	{
		#ifndef NDEBUG
		std::cout << "Depth: " << n->gn;
		std::cout << " Backtrack!" << std::endl;
		#endif
		C1_failed++;
		delete n;
		delete prop;
		return NULL; // dead--end
	}

	expanded++;
	#ifndef NDEBUG
	std::cout << "Depth: " << n->gn;
	std::cout << " Expanding with branching factor " << R.size() << ": " << std::endl;
	#endif

	if ( R.size() > 1 )
	{
		for ( unsigned k = 0; k < R.size(); k++ )
		{
			Dfs_Node* n2 = n->successor( R[k] );
			LH.compute(n2);
			float h = LH.metric_eval( goal->atom_vec() );
			m_c2_scores[R[k]] = std::add( h, n2->gn );
			delete n2;	
		}

		std::sort( R.begin(), R.end(), Sort_By_Score<float>(m_c2_scores) );
	}

	#ifndef NDEBUG
	std::cout << "Depth: " << n->gn;
	std::cout << " Minimal applicable actions scoring: " << std::endl;
	for ( unsigned i = 0; i < R.size(); i++ )
	{
		std::cout << "\t";
		m_task.print_operator( m_task.useful_ops()[ R[i] ], std::cout );
		std::cout << " score: " << m_c2_scores[R[i]] << std::endl;
	}
	#endif

	unsigned branches = 0;
	for( unsigned k = 0; k < R.size(); k++ )
		if ( R[k] != m_task.start() && m_c2_scores[R[k]] != std::numeric_limits<unsigned>::max() ) branches++;
	avg_branching_factor = avg_branching_factor + ( (1.0 / ((float)evaluated) ) * ((float)branches - avg_branching_factor));

	bool all_failed = true;
	for (unsigned i = 0; i < R.size(); i++ )
	{
		if ( R[i] == m_task.start() ||  m_c2_scores[R[i]] == std::numeric_limits<float>::max()  ) continue;
		#ifndef NDEBUG
		std::cout << "Depth: " << n->gn << " Considering action: ";
		m_task.print_operator( m_task.useful_ops()[ R[i]], std::cout );
		std::cout << std::endl;
		#endif
		Dfs_Node* c = n->successor( R[i] );
		if ( closed(c) == NULL )
		{

			if ( prop->find_first_ccc(n, R[i])  )
			{
				#ifndef NDEBUG
				std::cout << "Depth: " << n->gn << " Causal Chain found: " << std::endl;
				prop->Current_Chain.print(std::cout);
				std::cout << std::endl;
				#endif
				if ( prop->can_continue_with( Tail ) )
				{
					all_failed = false;
					Causal_Chain New_Tail;
					if ( Tail.body.empty() && Tail.a0 == m_task.end() )
						New_Tail = prop->Current_Chain;
					else
						New_Tail = Tail;
					#ifndef NDEBUG
					std::cout << "Depth: " << n->gn << " applying action: ";
					m_task.print_operator( m_task.useful_ops()[R[i]], std::cout );
					std::cout << std::endl;	
					#endif
					if ( R[i] == New_Tail.a0 )
						New_Tail.progress();
					if ( !closed(n) ) close( n );
					Dfs_Node* r = do_search( c, New_Tail );
					if ( r != NULL ) return r;
					break;
				}
			}
		}
		else
		{
			#ifndef NDEBUG
			std::cout << "Depth: " << n->gn;
			std::cout << " Duplicate!" << std::endl;
			#endif
			duplicates++;
			delete c;	
		}
	}

	if ( all_failed && Tail.a0 != m_task.end() )
	{
		#ifndef NDEBUG
		std::cout << "Depth: " << n->gn <<  " Could not find a follow-up for the tail:" << std::endl;
		Tail.print( std::cout );
		std::cout << std::endl; 
		std::cout << "Continuing the search discarding previous tail" << std::endl;
		#endif
		Tail.body.clear();
		Tail.a0 = m_task.end();
		Dfs_Node* r = do_search( n, Tail );
		if ( r!= NULL ) return r;
	}

	delete prop;
	return NULL;
}


template <typename Propagator_Type>
inline void Hill_Climbing<Propagator_Type>::close( Dfs_Node* n )
{
	unsigned h = m_closed.compute_hash( n->s->atom_vec() );
	assert( m_closed.get_element(h,n) == NULL );
	m_closed.add_element( h, n );
}

template <typename Propagator_Type>
inline Dfs_Node* Hill_Climbing<Propagator_Type>::closed( Dfs_Node* n )
{
	unsigned h = m_closed.compute_hash( n->s->atom_vec() );
	return m_closed.get_element(h,n);
}

template <typename Propagator_Type>
inline void Hill_Climbing<Propagator_Type>::remove( Dfs_Node* n )
{
	unsigned h = m_closed.compute_hash( n->s->atom_vec() );
	m_closed.remove_element(h,n);
}

}


#endif
