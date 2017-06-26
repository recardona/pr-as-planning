#ifndef __NFF_EHC__
#define __NFF_EHC__


#include "nff_dfs_node.hxx"
#include "nff.hxx"
#include "PDDL.hxx"
#include "pddl_fluent_table.hxx"
#include "nff_algobase.hxx"
#include "nff_options.hxx"
#include "nff_hsa_best_supporter.hxx"
#include "nff_harp_adapter.hxx"
#include "nff_helpful.hxx"
#include <queue>

namespace NFF
{

class Enforced_Hill_Climbing : public DFS_Skeleton
{
public:
	typedef	PDDL::Fluent_Set_Hashtable<Dfs_Node>				Closed_List;
	struct Node_Comparer
	{
		typedef Dfs_Node*	value_type;

		bool operator()( Dfs_Node* a, Dfs_Node* b )
		{
			if ( a->fn == b->fn )
			{
				unsigned a_score = a->num_obs;// + (a->parent ? a->parent->num_obs_in_h : 0);
				unsigned b_score = b->num_obs;// + (b->parent ? b->parent->num_obs_in_h : 0);
				return a_score < b_score;
				//return (a->num_obs < b->num_obs);//(a->num_obs + a->num_obs_in_h) < (b->num_obs + b->num_obs_in_h);
			}
			return a->fn > b->fn;
		}
	};


	typedef std::priority_queue<Dfs_Node*, std::vector<Dfs_Node*>, Node_Comparer>	Sorted_Open_List;
	
public:
	Enforced_Hill_Climbing()
	: m_task( PDDL::Task::instance() ), m_closed( 8192 )
	{
		NFF::BestSupporter *bs = new NFF::hsaBestSupporter;
		NFF::RPHA_w_mutexes *hae = new NFF::RPHA_w_mutexes(*bs);
		LH = new NFF::HARPHeuristicAdapter(bs, hae);
	}

	~Enforced_Hill_Climbing()
	{
	}

	bool 	  solve();
	void	  eval( Dfs_Node* n, std::vector<unsigned>& helpful );

	Dfs_Node* uniform_cost_search( Dfs_Node* initial );
	Dfs_Node* uniform_cost_search_no_pruning( Dfs_Node* initial );

protected:

	void 		close( Dfs_Node* n );
	Dfs_Node* 	closed( Dfs_Node* n );
	void		remove( Dfs_Node* n );
	void		print_open();
	bool		op_in_plan( Dfs_Node* n, unsigned op );

	PDDL::Task&			m_task;
	Sorted_Open_List		m_sorted_open;
	Closed_List			m_closed;
	State*				goal;
	NFF::HARPHeuristicAdapter*	LH;
	std::vector<unsigned>		m_helpful;

};

inline void	Enforced_Hill_Climbing::eval( Dfs_Node* n, std::vector<unsigned>& helpful )
{
	#ifndef NDEBUG
	std::cout << "Evaluating heuristic for expansion: ";
	m_task.print_operator( n->op, std::cout );
	#endif
	helpful.clear();
	Operator_Vec helpful_1;
	n->hn = LH->eval( *(n->s), helpful, path_found );
	n->num_obs_in_h = LH->num_obs_accounted();
	
	expanded++;
	if ( n->hn == std::numeric_limits<float>::max() )
		return;
	Operator_Vec& related_obs_ops = m_task.get_obs_for( n->op );
	if ( !related_obs_ops.empty() )
	{
		#ifndef NDEBUG
		std::cout << "Checking for possible insertions of forgoes due to operator ";
		m_task.print_operator( n->op, std::cout );
		std::cout << "( " << related_obs_ops.size() << " checks are to be performed )" << std::endl;
		#endif
		State* orig_state = n->s;
		bool state_changed = false;
		for ( unsigned i = 0; i < related_obs_ops.size(); i++ )
		{
			unsigned obs_op = related_obs_ops[i];
			PDDL::Operator* obs_op_ptr = m_task.useful_ops()[ obs_op ];
			if ( op_in_plan( n, obs_op ) ) continue;
			#ifndef NDEBUG
			std::cout << "\tPossible substitution: ";
			m_task.print_operator( obs_op, std::cout );
			std::cout << std::endl;
			#endif	
			State* sp = new State( *orig_state );
			sp->atom_vec().push_back( obs_op_ptr->explains() );
			sp->atom_set().set( obs_op_ptr->explains() );
			Dfs_Node* dummy = new Dfs_Node;
			dummy->s = sp;
			if ( closed(dummy) != NULL )
			{
				#ifndef NDEBUG
				std::cout << "Already in closed!" << std::endl;
				std::cout << "State: ";
				dummy->s->print( std::cout );
				std::cout << std::endl;
				std::cout << "Found in CLOSED: ";
				closed(dummy)->s->print(std::cout);
				std::cout << std::endl;
				#endif
				delete sp;
				continue;
			} 
			#ifndef NDEBUG
			std::cout << " to account for: ";
			m_task.print_fluent( obs_op_ptr->explains(), std::cout );
			std::cout << std::endl;
			#endif
			Operator_Vec helpful_2;
			float new_h = LH->eval( *sp, helpful_2, path_found );
			unsigned new_num_obs_act =  LH->num_obs_accounted();
			#ifndef NDEBUG
			std::cout << "Without introducing forgo: " << LH->num_obs_accounted() << "(" << n->hn << ")";
			std::cout << "Introducing the forgo: " << new_num_obs_act << "(" << new_h << ")" << std::endl;
			#endif
			if ( new_h <= n->hn && new_num_obs_act > n->num_obs_in_h )
			{
				state_changed = true;
				n->gn++; // Increase cost since obs cannot longer be accounted for
				n->hn = new_h;
				n->num_obs_in_h = new_num_obs_act;
				if ( n->s != orig_state )
					delete n->s;
				n->s = sp;
				helpful = helpful_2;
			}
			else
				delete sp;
		}
		if ( state_changed )
		{	
			Dfs_Node* dummy = new Dfs_Node;
			dummy->s = orig_state;
			//close( dummy );	
		 	//delete orig_state;
		}
	}

	//n->s->print(std::cout); std::cout << std::endl;
}

inline bool	Enforced_Hill_Climbing::solve()
{
	std::vector<unsigned> p;
	Dfs_Node* n = Dfs_Node::root();	
	goal = State::make_goal_state();	
	while ( !n->s->is_goal() )
	{
		#ifndef NDEBUG
		std::cout << "Depth: " << n->gn << std::endl;
		std::cout << "Starting EHC from state:" << std::endl;
		n->s->print( std::cout );
		std::cout << std::endl;
		std::cout << "Plan so far: " << std::endl;
		for ( unsigned k = 0; k < path_found.size(); k++ )
		{
			m_task.print_operator( path_found[k], std::cout );
			std::cout << " ";
			if ( k > 0 && k % 4 == 0 ) std::cout << std::endl;
		}
		std::cout << std::endl;
		#endif

		Dfs_Node* np = uniform_cost_search(n);
		if ( np == NULL )
		{
			#ifndef NDEBUG
			std::cout << "EHC failed" << std::endl;
			std::cout << "Plan so far: " << std::endl;
			for ( unsigned k = 0; k < path_found.size(); k++ )
			{
				m_task.print_operator( path_found[k], std::cout );
				std::cout << " ";
				if ( k > 0 && k % 4 == 0 ) std::cout << std::endl;
			}
			std::cout << std::endl;
			#endif 
			return false;
		}
		Dfs_Node* n2 = np;
		while ( n2 != n )
		{
			p.push_back( n2->op );
			n2 = n2->parent;
		}
		for ( int k = p.size()-1; k >= 0; k-- )
			path_found.push_back( p[k] );
		p.clear();
		n = np;
	} //while( n->hn > 0 );
	
	return true;
}

inline Dfs_Node*	Enforced_Hill_Climbing::uniform_cost_search( Dfs_Node* n0 )
{

	while ( !m_sorted_open.empty() )
	{
		Dfs_Node* n = m_sorted_open.top();
		m_sorted_open.pop();
		delete n;
	}
	m_closed.clear();
	n0->parent = NULL;
	eval( n0, m_helpful );
	#ifndef NDEBUG
	std::cout << "EHC: BFS Starts" << std::endl;
	std::cout << "Heuristic: " << n0->hn << " Potential explanations:" <<  n0->num_obs_in_h<< std::endl;
	std::cout << "Helpful Actions: " << std::endl;
	for ( unsigned i = 0; i < m_helpful.size(); i++ )
	{
		m_task.print_operator( m_helpful[i], std::cout );
		if ( i % 3 == 0 ) std::cout << std::endl;
	}
	std::cout << std::endl;
	#endif

	for ( unsigned i = 0; i < m_helpful.size(); i++ )
		m_sorted_open.push( n0->successor( m_helpful[i] ) );

	#ifndef NDEBUG
	print_open();
	#endif

	close(n0);
	Dfs_Node* best_exit = NULL;

	while ( !m_sorted_open.empty() )
	{
		Dfs_Node* n = m_sorted_open.top(); 
		m_sorted_open.pop();
		if ( closed(n) != NULL ) 
		{
			delete n; continue;
		}
		// Stop the search along those paths whose g(n) -- asuming uniform
		// costs -- is greater than that of the path corresponding to
		// the currently selected exit point
		//if ( best_exit != NULL && n->gn > best_exit->gn )
		if ( best_exit != NULL && n->depth > best_exit->depth )
		{
			delete n; continue;
		}
		#ifndef NDEBUG
		std::cout << "Evaluating state - g(n) = "<< n->gn << ", |A\\^o \\cap \\pi(n)| = " << n->num_obs << ": ";
		n->s->print(std::cout);
		std::cout << std::endl;
		#endif
		eval(n, m_helpful);
		assert( closed(n) == NULL );
		#ifndef NDEBUG
		std::cout << "Heuristic: " << n->hn << " Potential explanations in h:" <<  n->num_obs_in_h << std::endl;
		std::cout << "Helpful Actions: " << std::endl;
		for ( unsigned i = 0; i < m_helpful.size(); i++ )
		{
			m_task.print_operator( m_helpful[i], std::cout );
			if ( i % 3 == 0 ) std::cout << std::endl;
		}
		std::cout << std::endl;
		#endif

		if ( n->hn == std::numeric_limits<float>::max() ) // node is dead--end
		{
			delete n; continue;
		}

		if ( n->hn < n0->hn || n->num_obs > n0->num_obs  || n->s->is_goal() )
		{
			if ( best_exit == NULL )
			{
				best_exit = n;
				#ifndef NDEBUG
				std::cout << "Selected as exit point" << std::endl;
				#endif
			}
			else 
			{
				if ( (n->num_obs + n->num_obs_in_h) > (best_exit->num_obs + best_exit->num_obs_in_h) )
				{
					#ifndef NDEBUG
					Dfs_Node* c = closed(best_exit);
					if ( c != NULL )
					{
						std::cout << "Best exit node was in closed list!" << std::endl;
						std::cout << "Best Exit node: " << best_exit->gn << ", " << best_exit->hn << ", " << std::endl;
						std::cout << "State: "; best_exit->s->print(std::cout); std::cout << std::endl;
						std::cout << "Node found in closed: " << c->gn << ", " << c->hn << ", " << std::endl;
						std::cout << "State: "; c->s->print(std::cout); std::cout << std::endl;
						if ( *(c->s) == *(best_exit->s) )
						{
							std::cout << "State comparison function says they're the same" << std::endl;
						}

					}
					#endif
					close(best_exit);
					best_exit = n;
					#ifndef NDEBUG
					std::cout << "Selected as exit point" << std::endl;
					#endif
				}
				
			}
		}
		else
		{
			for ( unsigned i = 0; i < m_helpful.size(); i++ )
				m_sorted_open.push( n->successor( m_helpful[i] ) ); 
			#ifndef NDEBUG
			print_open();
			#endif

			close(n);
		}
		assert( !best_exit || closed(best_exit) == NULL );
	}
	#ifndef NDEBUG
	std::cout << std::endl;
	std::cout << "EHC: BFS Ends (goal found or heuristic improved!)" << std::endl;
	#endif
	return best_exit;	
}

inline void Enforced_Hill_Climbing::close( Dfs_Node* n )
{
	unsigned h = n->hash();//m_closed.compute_hash( n->s->atom_vec() );
	assert( m_closed.get_element(h,n) == NULL );
	m_closed.add_element( h, n );
}

inline Dfs_Node* Enforced_Hill_Climbing::closed( Dfs_Node* n )
{
	unsigned h = n->hash();//m_closed.compute_hash( n->s->atom_vec() );
	return m_closed.get_element(h,n);
}

inline void Enforced_Hill_Climbing::remove( Dfs_Node* n )
{
	unsigned h = n->hash();//m_closed.compute_hash( n->s->atom_vec() );
	m_closed.remove_element(h,n);
}

inline void Enforced_Hill_Climbing::print_open()
{
	std::vector<Dfs_Node*> open_vec;
	while (!m_sorted_open.empty() )
	{
		open_vec.push_back( m_sorted_open.top() );
		m_sorted_open.pop();
	}
	std::cout << "Contents of OPEN list: " << std::endl;
	unsigned ord = 1;
	for ( std::vector<Dfs_Node*>::iterator it = open_vec.begin();
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

inline bool Enforced_Hill_Climbing::op_in_plan( Dfs_Node* n, unsigned op )
{
	if ( n == NULL ) return false;
	Dfs_Node* current = n;
	while ( current != NULL )
	{
		if ( current->op == op ) return true;
		current = current->parent;
	}

	if ( std::find( path_found.begin(), path_found.end(), op ) != path_found.end() ) return true;

	return false;
}

}

#endif
