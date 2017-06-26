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
#ifndef __NFF_DFS_NODE__
#define __NFF_DFS_NODE__

#include "nff.hxx"
#include "nff_state.hxx"
#include "PDDL.hxx"
#include "jenkins_12bit.hxx"
#include <algorithm>

namespace NFF
{

class Dfs_Node
{
public:
	Dfs_Node();  
	~Dfs_Node();

	Dfs_Node* 			successor( unsigned op );
	Dfs_Node*			successor( unsigned op, Support& commit );
	Dfs_Node*               	relaxed_successor( unsigned op );
	void                    	apply_commit( unsigned op, Support& commit );
	static Dfs_Node*		root();
	bool				operator==( Dfs_Node& o )
	{
		return s->operator==(*(o.s)) && o.Pending == Pending;
	}
	
	bool compatible_with( unsigned op );

	size_t hash()
	{
		Atom_Vec& atoms = s->atom_vec();
		if ( atoms.empty() )
		{
			ub1 x = 0;
			unsigned h = jenkins_hash( &x, 1, 0 );
			return h;
		}
		std::sort( atoms.begin(), atoms.end() );
		unsigned h = jenkins_hash( (ub1*)(&atoms[0]), sizeof(unsigned), 0 );
		for ( unsigned i = 1; i < atoms.size(); i++ )
		{
			h = jenkins_hash( (ub1*)(&atoms[i]), sizeof(unsigned), h );
		}
		for ( Support_List::iterator i = Pending.begin(); i != Pending.end(); i++ )
		{
			unsigned p = i->p;
			h = jenkins_hash( (ub1*)&p, sizeof(unsigned), h );
			unsigned a = i->a;
			h = jenkins_hash( (ub1*)&a, sizeof(unsigned), h );
		}	
		return h;
	}

	// I leave them public for ease of access
	State*				s;	// state
	float				gn;	// accumulated cost
	float				hn;	// heuristic value
	float				fn;	// evaluation function
	unsigned			num_obs;
	unsigned			num_obs_in_h;
	Dfs_Node*			parent;
	unsigned			op;	// operator
	PDDL::Fluent_Set*		Locked;
	PDDL::Fluent_Set*		Waiting;
	PDDL::Fluent_Set*		Landmarks;
	Support_List			Pending;
	unsigned			depth;

protected:

	static	PDDL::Task&	sm_task;

};

}

#endif // nff_dfs_node.hxx
