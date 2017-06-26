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
#ifndef __NFF_PROP_UTILS__
#define __NFF_PROP_UTILS__

#include <vector>
#include <queue>
#include "nff_ext_state.hxx"

namespace NFF
{
enum Propagator_Node_Type
{
	ATOM = 0,
	OPERATOR,
	NONE
};

struct Reason
{
	int a;
	int q;
	int lq;

	bool operator==( const Reason& r )
	{
		return a == r.a && q == r.q && lq == r.lq;
	}
};

typedef std::vector<int>								Index_Vector;
typedef std::vector< std::pair< int, int> >						Index_Interval;
typedef std::vector< Reason >								Reason_Vec;
typedef std::vector< Reason_Vec >							Reasons_Vec;
typedef std::vector<Ext_State*>								Label_Vector;
typedef std::priority_queue<unsigned, std::vector<unsigned>, std::less<unsigned> >	Index_Queue_Desc;

class Propagator_Node
{
public:

	Propagator_Node();
	~Propagator_Node();

	Propagator_Node_Type		Type; // Node type
	int				Atom;
	int				Operator;
	Index_Vector			Parents;
	Index_Vector			Children;
	Label_Vector			Atom_Labels;
	Reasons_Vec			Reasons;
};

typedef std::vector<Propagator_Node*>	Propagation_Graph;


struct Propagator_Trace_Node
{
	Propagator_Node*	node;
	unsigned		index;
	Ext_State*		out_label;
};

typedef std::list< Propagator_Trace_Node* >	Propagation_Trace;

}

#endif // nff_prop_utils.hxx
