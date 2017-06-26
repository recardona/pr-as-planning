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
#include "nff_c3_bfs.hxx"
#include "nff_options.hxx"

namespace NFF
{

PDDL::Task& C3_Best_First_Search::sm_task = PDDL::Task::instance();

C3_Best_First_Search::C3_Best_First_Search()
	: expanded(0), m_closed( 8192 ) 
{
}

C3_Best_First_Search::~C3_Best_First_Search()
{
}

void	C3_Best_First_Search::eval( Node* n )
{
	estimator.compute( n->s );
	float est_cost_to_go = estimator.metric_eval(goal);
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

void	C3_Best_First_Search::dump_open()
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

Node*	C3_Best_First_Search::best_first_search( Node* n0 )
{
	assert( m_sorted_open.empty() );
	m_closed.clear();
	std::cout << "Initial node f(n) = " << n0->fn << ", g(n) = " << n0->gn << ", h(n) = " << n0->hn << std::endl;
	if ( n0->fn == std::numeric_limits<float>::max() )
	{
		std::cout << "Initial state heuristic is infinity!" << std::endl;
		std::cout << "Problem cannot be solved..." << std::endl;
		return NULL;
	}
	m_sorted_open.push( n0 );
	//dump_open();
	float min = std::numeric_limits<float>::max();
	
	while ( !m_sorted_open.empty() )
	{
		Node* n = m_sorted_open.top(); m_sorted_open.pop();
		//std::cout << "f(n) = " << n->fn << std::endl;
		if ( n->hn == 0 ) return n;
		Node* c = closed(n);
		if ( c != NULL ) // Node closed
		{
			delete n;
			continue;
		}

		if ( n->hn < min )
		{
			std::cout << "Advancing to distance: " << n->hn << "(" << expanded << " nodes expanded so far)" << std::endl;
			min = n->hn;
		}

		// n goes to CLOSED
		for ( unsigned i = 2; i < sm_task.useful_ops().size(); i++ )
		{
			if ( !n->s->can_apply(i) ) 
				continue;
			Node* n2 = n->successor( i );
			/*
			if ( is_mutex(n2) ) 
			{
				delete n2;
				continue;
			}
			*/
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
		if ( expanded % 1000 == 0 ) std::cout << expanded << std::endl;
		//dump_open();
	}

	return NULL;	
}

bool	C3_Best_First_Search::solve()
{
	Node* n = Node::root();	
	goal = State::make_goal_state();	
	eval( n );

	if ( n->hn == 0)
	{
		m_failed = false;
		return true;
	}

	Node* ng = best_first_search( n );
	if ( ng == NULL )
	{
		m_failed = true;
		return false;
	}
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

	return true;
}

}
