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
#ifndef __NFF__
#define __NFF__

#include <vector>
#include <set>
#include <list>
#include <map>
#include <queue>

#include "PDDL.hxx"
namespace NFF
{

typedef std::vector<unsigned>				Operator_Vec;
typedef std::set<unsigned>				Operator_Set;
typedef std::list<unsigned>				Operator_List;
typedef std::vector<unsigned>				Atom_Vec;
typedef std::set<unsigned>				Atom_Set;
typedef std::vector<bool>   				Bool_Vec;
typedef std::queue<unsigned>				Operator_Queue;

struct Atom_Pair
{
	unsigned	 	_atoms[2];
	static	unsigned	_nF;

	unsigned& 		p();
	unsigned& 		q();
	unsigned  		code();
	friend bool		intersects( Atom_Pair& lhs, Atom_Pair& rhs );
};

inline unsigned&	Atom_Pair::p() { return _atoms[0]; }

inline unsigned&	Atom_Pair::q() { return _atoms[1]; }

inline unsigned		Atom_Pair::code() { return p()*_nF + q(); }

inline unsigned		max_pair_code() { return Atom_Pair::_nF*Atom_Pair::_nF + Atom_Pair::_nF; }

inline bool		intersects( Atom_Pair& lhs, Atom_Pair& rhs )
{
	return (lhs.p() == rhs.p()) || (lhs.p() == rhs.q()) || (lhs.q() == rhs.p()) || (lhs.q() == rhs.q());
}

struct Support
{
	unsigned 		p;		// precondition
	unsigned 		a;		// supported
	static unsigned		num_ops;	// num of ops

	unsigned		index() const;
	bool			operator==( const Support& s ) const;
	bool			operator<( const Support& s ) const;
	bool			operator>( const Support& s ) const;
};

inline unsigned		Support::index() const 
{
	return p * num_ops + a;
}

inline bool		Support::operator==( const Support& s ) const
{
	return p == s.p && a == s.a;
}	

inline bool		Support::operator<( const Support& s ) const
{
	if ( p < s.p ) return true;
	if ( p == s.p && a < s.a ) return true;
	return false;	
}

inline bool		Support::operator>( const Support& s ) const
{
	if ( p > s.p ) return true;
	if ( p == s.p && a > s.a ) return true;
	return false;	
}

struct Possible_Support
{
	unsigned 	p;
	Operator_Vec	A;


	bool included( unsigned op ) const;
	bool operator==( const Possible_Support& s ) const;
	bool operator<( const Possible_Support& s ) const;
	bool operator>( const Possible_Support& s ) const;
};

inline bool Possible_Support::included( unsigned op ) const
{
	PDDL::Task& task = PDDL::Task::instance();
	for ( unsigned k = 0; k < A.size(); k++ )
		if ( task.equal_effects( A[k], op ) ) return true;
	return false;	
}

inline bool Possible_Support::operator==( const Possible_Support& s ) const
{
	return p == s.p && A == s.A;
}

inline bool Possible_Support::operator<( const Possible_Support& s ) const
{
	return p < s.p;
}

inline bool Possible_Support::operator>( const Possible_Support& s ) const
{
	return p > s.p;
}

typedef std::list<Possible_Support>	Possible_Support_List;

struct Causal_Link
{
	unsigned 	a2; 	// action requiring p
	unsigned 	p;	// precondition of a2 being supported
	unsigned 	a1;	// action adding p
	unsigned	score;

	static PDDL::Task&	task;
	
	unsigned	index() const;
	bool		operator==( const Causal_Link& other ) const;
};

inline unsigned		Causal_Link::index() const
{
	static unsigned offset_A =  task.useful_ops().size()
				 * task.fluents().size();
;
	static unsigned offset_B = task.useful_ops().size();

	return a1 * offset_A + p * offset_B + a2;
}

inline bool		Causal_Link::operator==( const Causal_Link& other) const
{
	return a2 == other.a2 && p == other.p && a1 == other.a1;
}

struct Ext_Causal_Link
{
	unsigned 		a;	// action adding p
	Possible_Support	S;	// actions requiring p
	unsigned		score;

	bool		operator==( const Ext_Causal_Link& other ) const;
};

inline bool	Ext_Causal_Link::operator==( const Ext_Causal_Link& other ) const
{
	return a == other.a && S == other.S;
}

class Causal_Link_Ranker
{
public:
	Causal_Link_Ranker() {}
	~Causal_Link_Ranker() {}

	bool operator()( const Causal_Link& a, const Causal_Link& b )
	{
		return a.score < b.score;
	}

	bool operator()( const Ext_Causal_Link& a, const Ext_Causal_Link& b )
	{
		return a.score < b.score;
	}

	bool operator()( const Ext_Causal_Link* a, const Ext_Causal_Link* b )
	{
		return a->score < b->score;
	}
};

typedef std::vector<Atom_Pair>				Atom_Pair_Vec;
typedef std::vector<Support>				Support_Vec;
typedef std::list<Support>				Support_List;
typedef std::vector<Causal_Link>			Causal_Link_Vec;
typedef std::vector<Ext_Causal_Link*>			Ext_CL_Ptr_Vec;

class Operator_Commitments : public std::map<unsigned, Support_Vec* >
{
typedef  std::map<unsigned, Support_Vec* > Container;

public:

	~Operator_Commitments();

	std::vector<Support>& operator[]( unsigned op )
	{
		Support_Vec* v = Container::operator[]( op );
		if ( v == NULL )
		{
			v = new Support_Vec;
			Container::operator[]( op ) = v;
		}
		return (*v);
	}
};

template <typename T>
class Sort_By_Score
{
public:
	Sort_By_Score( std::vector<T>& scores, bool asc = true )
		: m_scoring_func( scores ), m_asc_order( asc )
	{
	}
	
	Sort_By_Score( const Sort_By_Score& other )
		: m_scoring_func( other.m_scoring_func ), m_asc_order( other.m_asc_order )
	{
	}

	~Sort_By_Score( )
	{
	}

	bool operator()( unsigned x, unsigned y )
	{
		return ( m_asc_order ? m_scoring_func[x] < m_scoring_func[y] : m_scoring_func[x] > m_scoring_func[y]);
	}

protected:

	std::vector<T>&		m_scoring_func;
	bool				m_asc_order;
};

template <typename Heuristic>
class Sort_CL_By_Tail_H_Max
{
public:
	Sort_CL_By_Tail_H_Max( Heuristic& h, bool asc = true )
		: m_heuristic(h), m_asc_order( asc )
	{
	}
	
	Sort_CL_By_Tail_H_Max( const Sort_CL_By_Tail_H_Max& other )
		: m_heuristic( other.m_heuristic ), m_asc_order( other.m_asc_order )
	{
	}

	~Sort_CL_By_Tail_H_Max( )
	{
	}

	bool operator()( const Causal_Link& x, const Causal_Link& y )
	{
		return ( m_asc_order ? m_heuristic.value_op(x.a2) < m_heuristic.value_op(y.a2) : m_heuristic.value_op(x.a2) > m_heuristic.value_op(y.a2));
	}

protected:

	Heuristic&			m_heuristic;
	bool				m_asc_order;
};


struct Operator_Reason
{
	// Operator a needs to be applied in order to obtain p
	unsigned a;
	unsigned p;

	bool operator==( const Operator_Reason& r )
	{
		return a == r.a && p == r.p;
	}

	bool operator<( const Operator_Reason& r )
	{
		return ( a < r.a ) || ( a >= r.a && p < r.p );
	}
	
	bool operator>( const Operator_Reason& r )
	{
		return !(*this < r);
	}
};

typedef std::vector<Operator_Reason>	Operator_Reason_Vec;

class Operator_Reasons : public std::map<unsigned, Atom_Vec* >
{
typedef  std::map<unsigned, Atom_Vec* > Container;

public:

	~Operator_Reasons();

	Atom_Vec& operator[]( unsigned op )
	{
		Atom_Vec* v = Container::operator[]( op );
		if ( v == NULL )
		{
			v = new Atom_Vec;
			Container::operator[]( op ) = v;
		}
		return (*v);
	}
};

struct Causal_Chain
{
	unsigned	a0;
	Support_List	body;

	bool operator==( const Causal_Chain& o )
	{
		return a0 == o.a0 && body == o.body;
	}

	void 		print( std::ostream& os );
	void 		append( Causal_Chain& other );
	Support		progress();
};

inline void Causal_Chain::append( Causal_Chain& other )
{
	if (other.body.empty()) return;
	assert( body.back().a == other.a0 );
	for ( Support_List::iterator it = other.body.begin();
		it != other.body.end(); it++ )
		body.push_back( *it );
}

inline Support Causal_Chain::progress()
{
	Support first = body.front();
	body.pop_front();
	a0 = first.a;
	return first;	
}

struct Task
{
	unsigned	target_op;
	Causal_Chain	tail;

	void print( std::ostream& os );
};

}

#endif // nff.hxx
