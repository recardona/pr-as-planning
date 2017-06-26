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
#include "nff.hxx"

namespace NFF
{

unsigned Support::num_ops = PDDL::Task::instance().useful_ops().size();

PDDL::Task& Causal_Link::task = PDDL::Task::instance();

Operator_Commitments::~Operator_Commitments()
{
	for ( Container::iterator i = begin(); i != end(); i++ )
		delete i->second; // destroy vector of supports	
}

Operator_Reasons::~Operator_Reasons()
{
	for ( Container::iterator i = begin(); i != end(); i++ )
		delete i->second; // destroy vector of reasons
}

void Causal_Chain::print( std::ostream& os )
{
	PDDL::Task& task = PDDL::Task::instance();
	task.print_operator( task.useful_ops()[a0], os );
	os << " -> ";
	for ( Support_List::iterator it = body.begin();
		it != body.end(); it++ )
	{
		task.print_fluent( task.fluents()[it->p], os );
		os << " -> ";
		task.print_operator( task.useful_ops()[it->a], os );
		os << " -> ";
	}
}

void Task::print( std::ostream& os )
{
	PDDL::Task& the_task = PDDL::Task::instance();
	os << "TASK: Make operator executable: ";
	the_task.print_operator( the_task.useful_ops()[ target_op ], os );
	os << std::endl;
	os << "Chain to be continued: " << std::endl;
	os << "\t";
	tail.print( os );
	os << std::endl;
}

}
