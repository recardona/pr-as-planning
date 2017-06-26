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
#ifndef __NFF_EXT_STATE__
#define __NFF_EXT_STATE__

#include "PDDL.hxx"
#include "nff.hxx"
#include "nff_state.hxx"
#include "nff_logic.hxx"
#include <vector>

namespace NFF
{

class Ext_State // STRIPS with negation state
{
public:
	explicit Ext_State();
	Ext_State( std::vector<unsigned>& atoms, bool assume_false = true );
	Ext_State( State& s );
	Ext_State( Ext_State& s );
	~Ext_State();

	void update( unsigned op );
	void update_wo_neg( unsigned op );
	void assign( Atom_Vec& a, lbool v );
	void unassign( Atom_Vec& a );
	// returns true if union is a consistent state
	// false otherwise
	bool union_of( Ext_State& s1, Ext_State& s2 );
	bool union_with( Ext_State& s1); // inplace
	void intersection_of( Ext_State& s1, Ext_State& s2 );
	void intersection_with( Ext_State& s1); // inplace
	void intersection_with( PDDL::Fluent_Set& atoms ); // inplace
	bool included( Atom_Vec& a );

	lbool& operator[]( unsigned p ) { return m_model[p]; }
	bool  operator==( Ext_State& s );
	bool  operator!=( Ext_State& s )
	{
		return !(*this).operator==(s);
	}
	bool    empty();
	void	print();
protected:

	PDDL::Task&		task();

protected:

	std::vector<lbool>	m_model;
	static PDDL::Task&	sm_task;
};

inline PDDL::Task&		Ext_State::task()
{
	return sm_task;
}

}

#endif // nff_ext_state.hxx
