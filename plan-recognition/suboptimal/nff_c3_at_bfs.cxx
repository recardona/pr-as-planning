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
#include "nff_c3_at_bfs.hxx"
#include "nff_options.hxx"

namespace NFF
{

PDDL::Task& C3_Anytime_Best_First_Search::sm_task = PDDL::Task::instance();

C3_Anytime_Best_First_Search::C3_Anytime_Best_First_Search()
	: expanded(0), UB( std::numeric_limits<float>::max() ), num_sol(0), mutex_pruned(0), m_closed( 8192 )
{
}

C3_Anytime_Best_First_Search::~C3_Anytime_Best_First_Search()
{
}

void	C3_Anytime_Best_First_Search::eval( Node* n )
{
	estimator.compute( n->s );
	float est_cost_to_go = estimator.metric_eval(goal);
	//float est_cost_to_go = estimator->eval( *(n->s) );
	if ( est_cost_to_go == std::numeric_limits<float>::max() )
	{
		n->hn = est_cost_to_go;
		n->fn = est_cost_to_go;
		return;
	}
	n->hn = est_cost_to_go;
	
	//n->fn = std::add( n->gn, 5.0f*n->hn);
	n->fn = std::add( n->gn, n->hn );
}

void	C3_Anytime_Best_First_Search::dump_open()
{
	std::cout << "OPEN = {";
	std::vector<Node*> contents;
	while (!m_sorted_open.empty())
	{
		contents.push_back( m_sorted_open.top() );
		m_sorted_open.pop();
	}
	for ( unsigned i = 0; i < contents.size(); i++ )
	{
		std::cout << "f(n) = " << contents[i]->fn;
		if ( i < contents.size()-1 ) std::cout << ", ";
		m_sorted_open.push( contents[i] );
	}
	std::cout << "}" << std::endl;
}

Node*	C3_Anytime_Best_First_Search::best_first_search()
{
	
	while ( !m_sorted_open.empty() )
	{
		Node* n = m_sorted_open.top(); m_sorted_open.pop();
		//std::cout << "f(n) = " << n->fn << std::endl;
		Node* c = closed(n);
		if ( c != NULL ) // Node closed
		{
			delete n;
			continue;
		}
		if ( n->gn >= UB )
		{
			delete n;
			continue;
		}
		/*
		if ( is_mutex(n) ) 
		{
			delete n;
			mutex_pruned++;
			continue;
		}
		*/
		if ( n->hn == 0 )
		{
			UB = n->gn;
			std::cout << "Solution found with cost: " << n->gn << std::endl;
			return n;
		}

		// n goes to CLOSED
		for ( unsigned i = 2; i < sm_task.useful_ops().size(); i++ )
		{
			if ( !n->s->can_apply(i) ) 
				continue;
			Node* n2 = n->successor( i );
			eval(n2);
			if ( n2->fn == std::numeric_limits<unsigned>::max() )
			{
				delete n2;
				continue;
			}
			m_sorted_open.push( n2 );
		}
		expanded++;
		close(n);
		if ( expanded % 1000 == 0 ) std::cout << "Nodes expanded so far: " << expanded << std::endl;
		//dump_open();
	}

	return NULL;	
}

bool	C3_Anytime_Best_First_Search::solve()
{
	Node* n = Node::root();	
	goal = State::make_goal_state();	
	eval( n );

	if ( n->hn == 0)
	{
		m_failed = false;
		return true;
	}

	assert( m_sorted_open.empty() );
	m_closed.clear();
	std::cout << "Initial node f(n) = " << n->fn << ", g(n) = " << n->gn << ", h(n) = " << n->hn << std::endl;
	if ( n->fn == std::numeric_limits<float>::max() )
	{
		std::cout << "Initial state heuristic is infinity!" << std::endl;
		std::cout << "Problem cannot be solved..." << std::endl;
		return NULL;
	}
	m_sorted_open.push( n );

	Node* ng = best_first_search();

	while ( ng != NULL )
	{
		std::vector<unsigned> path_rev;

		Node* n2 = ng;
		while ( n2 != n )
		{
			path_rev.push_back( n2->op );
			n2 = n2->parent;
		}
		for ( int k = path_rev.size()-1; k>=0; k-- )
		{
			path_found.push_back( path_rev[k] );
		}
		// Write plan
		PDDL::Task& task = PDDL::Task::instance();
		std::ofstream plan_out( "plan.solution" );
		unsigned action_count = 1;
		for ( unsigned k = 0; k < path_found.size(); k++ )
		{
			PDDL::Operator* op = task.useful_ops()[path_found[k]];
			plan_out << (float)action_count++ << ": ";
			task.print_operator( op, plan_out );
			plan_out << std::endl;
		}
		plan_out.close();

		ng = best_first_search();
	}

	return true;
}

}
