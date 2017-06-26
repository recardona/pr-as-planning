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
#ifndef __NFF_STATE__
#define __NFF_STATE__

#include "PDDL.hxx"
#include <vector>
#include "pddl_fluent_set.hxx"
#include <iostream>

namespace NFF
{

class State
{
public:
	explicit State();
	State( std::vector<unsigned>& atoms );
	State( State& s );
	~State();

	static State* make_initial_state();
	static State* make_goal_state();

	std::vector<unsigned>&	atom_vec();
	PDDL::Fluent_Set&	atom_set();

	void			print( std::ostream& os );
	void			print_indices( std::ostream& os );

	bool			is_goal();

	bool			can_apply( unsigned op_idx );
	State*			apply( unsigned op_idx );

	bool			operator==( State& o );
	std::vector<unsigned>&	enabled_atoms() { return m_enabled_atoms; }

protected:

	PDDL::Fluent_Set*	m_atomset;
	std::vector<unsigned>*	m_atoms;
	PDDL::Task&		task();

	static PDDL::Task&	sm_task;
	std::vector<unsigned>	m_enabled_atoms;
};

inline PDDL::Task&		State::task()
{
	return sm_task;
}

inline std::vector<unsigned>&	State::atom_vec()
{
	return *m_atoms;
}

inline PDDL::Fluent_Set&	State::atom_set()
{
	return *m_atomset;
}

inline bool State::operator==( State& o )
{
	if ( o.atom_vec().size() != atom_vec().size() )
		return false;
	
	for ( unsigned i = 0; i < atom_vec().size(); i++ )
		if ( !o.atom_set().isset( atom_vec()[i] ) ) return false;
	return true;
}


}

#endif // nff_state.hxx
