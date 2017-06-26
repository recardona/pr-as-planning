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
#include "nff_bfs.hxx"
#include "nff_options.hxx"

namespace NFF
{

PDDL::Task& Best_First_Search::sm_task = PDDL::Task::instance();

Best_First_Search::Best_First_Search()
	: expanded(0), m_closed( 8192 ) 
{
	NFF::BestSupporter *bs = new NFF::hsaBestSupporter;
	NFF::HelpfulActionExtractor *hae = new NFF::PlanActions(*bs);
	estimator = new NFF::HARPHeuristicAdapter(bs, hae);
}

Best_First_Search::~Best_First_Search()
{
}

bool	Best_First_Search::op_in_plan( Node* n, unsigned op )
{
	if ( n == NULL ) return false;
	Node* current = n;
	while ( current != NULL )
	{
		if ( current->op == op ) return true;
		current = current->parent;
	}

	return false;
}

void	Best_First_Search::extract_plan( Node* n, Operator_Vec& plan )
{
	Node* current = n;
	while ( current != NULL )
	{
		plan.push_back( current->op );
		current = current->parent;
	}
}

void	Best_First_Search::eval( Node* n )
{
	Operator_Vec helpful;
	Operator_Vec partial_plan;
	extract_plan( n, partial_plan );

	float est_cost_to_go = estimator->eval( *(n->s), helpful, partial_plan );

	n->hn = est_cost_to_go;
	n->num_obs_in_h = estimator->num_obs_accounted();	
	n->fn = std::add( n->gn, n->hn );
	if ( n->fn == std::numeric_limits<float>::max() )
		return;

	Operator_Vec& related_obs_ops = sm_task.get_obs_for( n->op );
	if ( !related_obs_ops.empty() )
	{
		#ifndef NDEBUG
		std::cout << "Checking for possible insertions of forgoes due to operator ";
		sm_task.print_operator( n->op, std::cout );
		std::cout << "( " << related_obs_ops.size() << " checks are to be performed )" << std::endl;

		#endif
		State* orig_state = n->s;
		bool state_changed = false;
		for ( unsigned i = 0; i < related_obs_ops.size(); i++ )
		{
			unsigned obs_op = related_obs_ops[i];
			PDDL::Operator* obs_op_ptr = sm_task.useful_ops()[ obs_op ];
			if ( op_in_plan( n, obs_op ) ) continue;
			#ifndef NDEBUG
			std::cout << "\tPossible substitution: ";
			sm_task.print_operator( obs_op, std::cout );
			#endif	
			State* sp = new State( *orig_state );
			sp->atom_vec().push_back( obs_op_ptr->explains() );
			sp->atom_set().set( obs_op_ptr->explains() );
			Node* dummy = new Node;
			dummy->s = sp;
			if ( closed(dummy) != NULL )
			{
				#ifndef NDEBUG
				std::cout << "Already in closed!" << std::endl;
				#endif
				delete sp;
				continue;
			} 
			#ifndef NDEBUG
			std::cout << " to account for: ";
			sm_task.print_fluent( obs_op_ptr->explains(), std::cout );
			std::cout << std::endl;
			#endif
			float new_h = estimator->eval( *sp, helpful, partial_plan );
			unsigned new_num_obs_act =  estimator->num_obs_accounted();
			#ifndef NDEBUG
			std::cout << "Without introducing forgo: " << estimator->num_obs_accounted() << "(" << n->hn << ")";
			std::cout << "Introducing the forgo: " << new_num_obs_act << "(" << new_h << ")" << std::endl;
			#endif
			if ( new_h <= n->hn && new_num_obs_act > n->num_obs_in_h )
			{
				state_changed = true;
				n->hn = new_h;
				n->num_obs_in_h = new_num_obs_act;
				if ( n->s != orig_state )
					delete n->s;
				n->s = sp;
			}
			else
				delete sp;
		}
		if ( state_changed )
		{
			Node* dummy = new Node;
			dummy->s = orig_state;
			//close( dummy );
			//delete orig_state;
		}
	}
	
}

void	Best_First_Search::dump_open()
{
	std::vector<Node*> open_vec;
	while (!m_sorted_open.empty() )
	{
		open_vec.push_back( m_sorted_open.top() );
		m_sorted_open.pop();
	}
	std::cout << "Contents of OPEN list: " << std::endl;
	unsigned ord = 1;
	for ( std::vector<Node*>::iterator it = open_vec.begin();
		it != open_vec.end(); it++ )
	{
		std::cout << ord << ": ";
		std::cout << "(";
		std::cout << (*it)->gn;
		std::cout << ",";
		std::cout << (*it)->hn;
		std::cout << ",";
		std::cout << (*it)->num_obs;
		std::cout << "/";
		//std::cout << (*it)->num_obs + ((*it)->parent ? (*it)->parent->num_obs_in_h : 0);
		std::cout << ")";
		if ( ord % 4 == 0 ) std::cout << std::endl;
		ord++;
		m_sorted_open.push( *it );
	}
	std::cout << std::endl;

}

Node*	Best_First_Search::best_first_search( Node* n0 )
{
	assert( m_sorted_open.empty() );
	m_closed.clear();
	#ifndef NDEBUG
	std::cout << "Initial node f(n) = " << n0->fn << ", g(n) = " << n0->gn << ", h(n) = " << n0->hn << std::endl;
	#endif
	if ( n0->fn == std::numeric_limits<float>::max() )
	{
		std::cout << "Initial state heuristic is infinity!" << std::endl;
		std::cout << "Problem cannot be solved..." << std::endl;
		return NULL;
	}
	m_sorted_open.push( n0 );
	#ifndef NDEBUG
	dump_open();
	#endif
	float min = std::numeric_limits<float>::max();
	
	while ( !m_sorted_open.empty() )
	{
		Node* n = m_sorted_open.top(); m_sorted_open.pop();
		//std::cout << "f(n) = " << n->fn << std::endl;
		//if ( n->hn == 0 ) return n;
		if ( n->s->is_goal() ) return n;
		Node* c = closed(n);
		if ( c != NULL ) // Node closed
		{
			delete n;
			continue;
		}

		#ifndef NDEBUG
		if ( n->hn < min )
		{
			std::cout << "Advancing to distance: " << n->hn << "(" << expanded << " nodes expanded so far)" << std::endl;
			min = n->hn;
		}
		#endif

		// n goes to CLOSED
		for ( unsigned i = 2; i < sm_task.useful_ops().size(); i++ )
		{
			if ( !n->s->can_apply(i) ) 
				continue;
			if ( sm_task.is_obs(i) )
			{
				bool already_used = false;
				Node* p = n->parent;
				while ( p!= NULL )
				{
					if ( p->op == i )
					{
						already_used = true;
						break;
					}
					p = p->parent;
				}
				if (already_used) continue;
			}
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
		#ifndef NDEBUG
		if ( expanded % 1000 == 0 ) std::cout << expanded << std::endl;
		dump_open();
		#endif
	}

	return NULL;	
}

bool	Best_First_Search::solve()
{
	Node* n = Node::root();	
	goal = State::make_goal_state();	
	eval( n );

	if ( n->s->is_goal() ) 
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
