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
#include "nff_node.hxx"

namespace NFF
{

PDDL::Task&	Node::sm_task = PDDL::Task::instance();

Node::Node()
	: s(NULL), gn(0), hn(0), fn(0), parent(NULL), op(0), num_obs(0), num_obs_in_h(0)
{
}

Node::~Node()
{
	if ( s )
		delete s;
}

Node*		Node::root()
{
	Node*	n = new Node;
	n->s = State::make_initial_state();
	n->gn = 0;
	n->parent = NULL;
	n->op = 0;
	n->num_obs = 0;
	n->num_obs_in_h = 0;
	return n;
}

Node*		Node::successor( unsigned op )
{
	Node* succ = new Node;
	succ->gn = gn + sm_task.op_cost( op );
	succ->fn = succ->gn;
	succ->s = s->apply(op); // we need to have made sure previously that we're really going to be able to apply the operator
	succ->op = op;
	succ->parent = this;
	succ->num_obs = num_obs + ( sm_task.is_obs(op) ? 1 : 0);
	return succ;
}

}
