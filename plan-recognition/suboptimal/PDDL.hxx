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
#ifndef __PDDL__
#define __PDDL__

#include "pddl_basic_types.hxx"
#include "pddl_string_table.hxx"
#include "pddl_fluent_set.hxx"
#include "nff_logic.hxx"
#include <iostream>
#include <fstream>
#include <set>
#include <cassert>

namespace NFF
{

class H2;

}

namespace PDDL
{

class Fluent // A grounded predicate
{
public:

	explicit Fluent();
	Fluent( int code, bool is_explanation = false );
	~Fluent();

	int			code() { return m_code; }
	bool			is_explained() { return m_is_obs_explanation; }	
	void			enable() { m_enabled = true; }
	void			disable() { m_enabled = false; }
	bool			enabled() { return m_enabled; }

protected:
	int			m_code;
	bool			m_is_obs_explanation;
	bool			m_enabled;
};

class Operator
{
public:

	explicit Operator();
	Operator( int code );

	~Operator();

	Fluent_Set&		preconds() 	{ return *m_precondition; }
	Fluent_Set&		adds() 		{ return *m_adds; }
	Fluent_Set&		dels() 		{ return *m_dels; }
	Fluent_Set&		xdels()		{ return *m_xdels; }
	std::vector<unsigned>&	prec_vec()	{ return m_prec_vec; }
	std::vector<unsigned>&	add_vec()	{ return m_add_vec; }
	std::vector<unsigned>&	del_vec()	{ return m_del_vec; }
	std::vector<NFF::Lit>&	xdel_vec()	{ return m_xdel_vec; }
	
	int			code()		{ return m_code; }
	void			make_xdel_lists();
	float			metric_cost() { return m_metric; }
	void			set_metric_cost( float cost ) { m_metric = cost; }

	
	void			set_explained( unsigned f ) { m_provided_explanation = f; }
	unsigned		explains() { return m_provided_explanation; }
	void			set_req_explanation( unsigned f ) { m_req_explanation = f; }
	unsigned		explanation_req() { return m_req_explanation; }
	bool			accounts_obs() { return m_explains_obs; }
	bool			forgoes_obs() { return m_forgoes_obs; }

	static Operator*	new_regular_op( int code );
	static Operator*	new_explain_op( int code );
	static Operator*	new_forgo_op( int code );

protected:
	
	std::vector<unsigned>	m_prec_vec;
	std::vector<unsigned>	m_add_vec;
	std::vector<unsigned>	m_del_vec;
	std::vector<NFF::Lit>	m_xdel_vec;
	Fluent_Set*		m_precondition;
	Fluent_Set*		m_adds;
	Fluent_Set*		m_dels;
	Fluent_Set*		m_xdels;
	int			m_code;
	float			m_metric;
	bool			m_explains_obs;
	bool			m_forgoes_obs;
	unsigned		m_req_explanation;
	unsigned		m_provided_explanation;
};


class Task
{
	
private:
	Task();
public:
	~Task();
	static Task& instance();

	void setup();

	int 			fluent_count() { return (int)m_fluent_table.size(); }
	std::vector<Fluent*>&	fluents() { return m_fluent_table; }
	std::vector<Operator*>& useful_ops() { return m_useful_operator_table; }

	std::vector<unsigned>&	initial_state() { return m_initial_state; }
	std::vector<unsigned>& 	goal_state() { return m_goal_state; }

	unsigned		start() { return m_start_id; }
	unsigned		end() { return m_end_id; }
	// operators with fluent f in their preconditions
	std::vector<unsigned>&	required_by( unsigned f ) { return m_required_by[f]; }
	// operators with fluent f in their add list
	std::vector<unsigned>&	added_by( unsigned f ) { return m_added_by[f]; }
	// operators with fluent f in their del list
	std::vector<unsigned>&	deleted_by( unsigned f ) { return m_deleted_by[f]; }
	// operators that e-delete f ( either q \in Add(o), h²( f, q ) = inf, or r \in Pre(o), r \notin Add(o)
	// h²(p,r) = inf
	// NOTE: This info becomes available through the seq_h2 or par_h2 compute() methods.
	std::vector<unsigned>&	e_deleted_by( unsigned f ) { return m_e_deleted_by[f]; }
	PDDL::Fluent_Set&	fast_op_edeletes( unsigned op ) { return *(m_fast_op_edeletes[op]); }

	void			match_ops_and_obs();
	std::vector<unsigned>&	get_obs_for( unsigned op ) { return m_ops_to_obs[op]; }

	std::vector<unsigned>&	op_edeletes( unsigned op ) { return m_op_edeletes[op]; }

	std::vector< std::vector<unsigned> >& landmarks_table() { return m_op_landmarks; }
	std::vector<unsigned>&  landmarks_for( unsigned op ) { return m_op_landmarks[op]; }
	

	bool			useful( unsigned op ) { return m_is_useful[op]; }
	bool			reachable( unsigned op ) { return m_is_reachable[op]; }
	bool			is_explained( unsigned f ) { return m_is_explained[f]; }
	bool			is_obs( unsigned op ) { return m_is_obs[op]; }
	bool			is_forgo( unsigned op ) { return useful_ops()[op]->forgoes_obs(); }
	bool			precedes_in_obs_seq( unsigned op1, unsigned op2 );
	void			sort_observations();
	unsigned		start_time_lb( unsigned op ) { return m_start_times_lb[op]; }
	std::vector<unsigned>&	start_time_lbs() { return m_start_times_lb; }

	void 			set_start_time_lb( std::vector<unsigned>& stimes );

	std::vector<unsigned>&	sorted_explanations() { return m_sorted_explanations; }

	void			print_fluents( std::ostream& os );
	void			print_fluent( Fluent* f, std::ostream& os );
	void			print_fluent( unsigned f, std::ostream& os )
	{
		assert( f > 0 && f < fluents().size() );
		print_fluent( fluents()[f], os );
	}

	void			print_operator( Operator* o, std::ostream& os );
	void			print_operator_full( Operator* o, std::ostream& os );
	void			print_operator( unsigned o, std::ostream& os )
	{
		assert( o < useful_ops().size() );
		print_operator( useful_ops()[o], os );
	}
	void			print_operators( std::ostream& os );

	void			print_initial_state( std::ostream& os );
	void			print_goal_state( std::ostream& os );

	String_Table&		str_tab() { return m_string_table; }

	NFF::H2&			h2() { return *m_h2; }
	void			set_h2( NFF::H2* h2 ) { m_h2 = h2; }
	
	bool			equal_effects( unsigned o1, unsigned o2 )
	{
		if ( o1 == o2 ) return true;
		Operator* o1ptr = useful_ops()[o1];
		Operator* o2ptr = useful_ops()[o2];
		
		if ( o1ptr->add_vec().size() != o2ptr->add_vec().size() ) return false;
		if ( o1ptr->del_vec().size() != o2ptr->del_vec().size() ) return false;

		for ( unsigned k = 0; k < o1ptr->add_vec().size(); k++ )
			if ( !o2ptr->adds().isset( o1ptr->add_vec()[k] ) ) return false;
		for ( unsigned k = 0; k < o2ptr->del_vec().size(); k++ )
			if ( !o2ptr->dels().isset( o1ptr->del_vec()[k] ) ) return false;
		
		return true;
	}

	float op_cost( unsigned op ) { return m_op_costs[op]; }
protected:
	void			create_fluents();
	void 			create_init_and_goal();
	void			create_operators();	

protected:
	std::vector< Fluent* >				m_fluent_table;
	std::vector< Operator* >			m_useful_operator_table;
	std::vector<unsigned>				m_initial_state;
	std::vector<unsigned>				m_goal_state;
	unsigned					m_start_id;
	unsigned					m_end_id;
	std::vector< std::vector<unsigned> >		m_required_by;
	std::vector< std::vector<unsigned> >		m_added_by;
	std::vector< std::vector<unsigned> > 		m_deleted_by;
	std::vector< std::vector<unsigned> > 		m_e_deleted_by;
	std::vector< PDDL::Fluent_Set* >		m_fast_op_edeletes;
	std::vector< std::vector<unsigned> >		m_op_edeletes;
	std::vector< std::vector<unsigned> >		m_op_landmarks;
	std::vector< unsigned >				m_explain_ops;
	std::vector<bool>				m_is_explained;
	std::vector<bool>				m_is_obs;
	std::vector<bool>				m_is_useful;
	std::vector<bool>				m_is_reachable;
	std::vector<unsigned>				m_start_times_lb;
	String_Table&					m_string_table;
	std::ofstream					m_glog;
	std::vector<bool>			 	m_negs_in_precs;
	NFF::H2*					m_h2;
	std::vector<float>				m_op_costs;
	std::vector< std::vector< unsigned> >		m_sorted_obs;
	std::vector<unsigned>				m_sorted_explanations;
	std::vector< std::vector<unsigned> >		m_ops_to_obs;
};

}
#endif // PDDL.hxx
