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
#ifndef __NNF_C3_AT_BFS__
#define __NNF_C3_AT_BFS__

#include "nff_node.hxx"
#include "nff_h_add_plus.hxx"
#include "nff_h2.hxx"
#include "PDDL.hxx"
#include "pddl_fluent_table.hxx"
#include <deque>
#include <queue>

namespace NFF
{

class C3_Anytime_Best_First_Search
{
public:

	C3_Anytime_Best_First_Search();
	~C3_Anytime_Best_First_Search();

	bool	solve();
	bool	failed();
	Node*	best_first_search(  );
	void	eval( Node* n );
	bool    is_mutex( Node* n );
	
protected:
	void 	close( Node* n );
	Node* 	closed( Node* n );
	void	remove( Node* n );
	void	dump_open();
public:
	// For ease of use (again)
	std::vector<unsigned>		path_found;
	unsigned			expanded;
	float				UB;
	unsigned			num_sol;
	unsigned			mutex_pruned;
protected:

	struct Node_Comparer
	{
		typedef Node*	value_type;

		bool operator()( Node* a, Node* b )
		{
			return a->fn > b->fn;
		}
	};

	typedef	PDDL::Fluent_Set_Hashtable<NFF::Node>				Closed_List;
	typedef std::priority_queue<Node*, std::vector<Node*>, Node_Comparer>	Sorted_Open_List;

	static PDDL::Task&		sm_task;
	Additive_Heuristic_Plus		estimator;
	H2                              h2_mutex;
	State*				goal;
	bool				m_failed;
	Sorted_Open_List		m_sorted_open;
	Closed_List			m_closed;
		
};

inline bool C3_Anytime_Best_First_Search::is_mutex( Node* n )
{
	h2_mutex.compute_only_mutexes(n->s);
	return h2_mutex( goal ) == std::numeric_limits<unsigned>::max();	
}

inline bool C3_Anytime_Best_First_Search::failed() { return m_failed; }

inline void C3_Anytime_Best_First_Search::close( Node* n )
{
	unsigned h = m_closed.compute_hash( n->s->atom_vec() );
	assert( m_closed.get_element(h) == NULL );
	m_closed.add_element( h, n );
}

inline Node* C3_Anytime_Best_First_Search::closed( Node* n )
{
	unsigned h = m_closed.compute_hash( n->s->atom_vec() );
	return m_closed.get_element(h,n);
}

inline void C3_Anytime_Best_First_Search::remove( Node* n )
{
	unsigned h = m_closed.compute_hash( n->s->atom_vec() );
	m_closed.remove_element(h,n);
}

}

#endif // nnf_ehc.hxx
