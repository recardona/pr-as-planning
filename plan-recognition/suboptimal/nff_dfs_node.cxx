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
#include "nff_dfs_node.hxx"

namespace NFF
{

PDDL::Task&	Dfs_Node::sm_task = PDDL::Task::instance();

Dfs_Node::Dfs_Node()
	: s(NULL), gn(0), hn(0), fn(0), num_obs(0), num_obs_in_h(0), parent(NULL), op(0), depth(0)
{
	Locked = new PDDL::Fluent_Set( sm_task.fluents().size() );
	Waiting = new PDDL::Fluent_Set( sm_task.useful_ops().size() );
	Landmarks = new PDDL::Fluent_Set( sm_task.useful_ops().size() );
}

Dfs_Node::~Dfs_Node()
{
	if ( s )
		delete s;
	if ( Locked )
		delete Locked;
	if ( Waiting )
		delete Waiting;
	if ( Landmarks )
		delete Landmarks;
}

Dfs_Node*		Dfs_Node::root()
{
	Dfs_Node*	n = new Dfs_Node;
	n->s = State::make_initial_state();
	n->gn = 0;
	n->depth = 0;
	n->parent = NULL;
	n->op = 0;
	n->num_obs = 0;
	n->num_obs_in_h = 0;
	return n;
}

Dfs_Node*		Dfs_Node::successor( unsigned op )
{
	Dfs_Node* succ = new Dfs_Node;
	succ->gn = gn + sm_task.op_cost( op );
	succ->fn = succ->gn;
	succ->depth = depth + 1;
	succ->s = s->apply(op); // we need to have made sure previously that we're really going to be able to apply the operator
	succ->op = op;
	succ->num_obs = num_obs + ( sm_task.is_obs(op) ? 1 : 0);
	succ->parent = this;
	return succ;
}

Dfs_Node* Dfs_Node::successor( unsigned op, Support& commit )
{
	Dfs_Node* succ = new Dfs_Node;
	succ->gn = gn + sm_task.op_cost( op );
	succ->fn = succ->gn;
	succ->s = s->apply(op);
	succ->op = op;
	succ->parent = this;

	// Propagating causal link commitments
	// First remove from Locked & Pending, corresponding atoms and supports
	for ( Support_List::iterator i = Pending.begin();
		i != Pending.end(); i++ )
	{
		// Copy supports where the supporter isn't op
		if ( !sm_task.equal_effects( i->a,  op) )
		{
			succ->Locked->set( i->p ); // add atom to succ Locked list
			succ->Waiting->set( i->a );
			succ->Pending.push_back( *i ); // add support to succ Pending list
		}
	} 

	// Now add op commitments to Locked & Pending
	if ( succ->Pending.empty() )
	{
		succ->Pending.push_back( commit );
		succ->Locked->set( commit.p );
		succ->Waiting->set( commit.a );
		return succ;
	}

	Support_List::iterator i = succ->Pending.begin();
	for ( ; i != succ->Pending.end(); i++ )
	{
		if ( (*i) > commit )
		{
			succ->Pending.insert(i, commit);
			succ->Locked->set( commit.p );
			succ->Waiting->set( commit.a );
			return succ;
		}	
	}
	if ( i == succ->Pending.end() )
	{
		succ->Pending.push_back( commit );
		succ->Locked->set( commit.p );
		succ->Waiting->set( commit.a );
	}

	return succ;	
}

bool Dfs_Node::compatible_with( unsigned op )
{
	bool pruned = false;
	PDDL::Operator* op_ptr = sm_task.useful_ops()[op];
	Atom_Vec& op_adds =  op_ptr->add_vec();
	for ( unsigned k = 0; k < !pruned && op_adds.size(); k++ )
	{
		if ( Locked->isset( op_adds[k] ) )
		{
			for ( Support_List::iterator j = Pending.begin();
				j != Pending.end(); j++ )
				if ( j->p == op_adds[k] && !sm_task.equal_effects(j->a, op ) )
				{
					pruned = true;
					//delete R[i];
					break;
				}
		}
	}
	Atom_Vec& op_dels = op_ptr->del_vec();
	for ( unsigned k = 0;  !pruned && k < op_dels.size(); k++ )
	{
		if ( Locked->isset( op_dels[k] ) )
		{
			for ( Support_List::iterator j = Pending.begin();
				j != Pending.end(); j++ )
				if ( j->p == op_dels[k] && !sm_task.equal_effects(j->a, op)  )
				{
					pruned = true;
					//delete R[i];
					break;
				}
		}
	}

	return pruned == false;
}

void Dfs_Node::apply_commit( unsigned op, Support& commit )
{
	
	// Propagating causal link commitments
	// First remove from Locked & Pending, corresponding atoms and supports
	for ( Support_List::iterator i = parent->Pending.begin();
		i != parent->Pending.end(); i++ )
	{
		// Copy supports where the supporter isn't op
		if ( !sm_task.equal_effects( i->a,  op) )
		{
			this->Locked->set( i->p ); // add atom to succ Locked list
			this->Waiting->set( i->a );
			this->Pending.push_back( *i ); // add support to succ Pending list
		}
	} 

	// Now add op commitments to Locked & Pending
	this->Pending.push_back( commit );
	this->Locked->set( commit.p );
	this->Waiting->set( commit.a );
	


}

Dfs_Node* Dfs_Node::relaxed_successor( unsigned op )
{
	Dfs_Node* succ = new Dfs_Node;
	succ->gn = gn + sm_task.op_cost( op );
	succ->fn = succ->gn;
	succ->s = s->apply(op);
	succ->op = op;
	succ->parent = this;

	// Propagating causal link commitments
	// First remove from Locked & Pending, corresponding atoms and supports
	for ( Support_List::iterator i = Pending.begin();
		i != Pending.end(); i++ )
	{
		// Copy supports where the supporter isn't op
		if ( !sm_task.equal_effects( i->a,  op) )
		{
			succ->Locked->set( i->p ); // add atom to succ Locked list
			succ->Waiting->set( i->a );
			succ->Pending.push_back( *i ); // add support to succ Pending list
		}
	} 


	return succ;	
}

}
